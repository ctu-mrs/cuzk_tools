import rospy

import numpy as np

from dmr5g import WGS_TO_SJTSK, get_utm_to_sjtsk_trans


class UnsupportedFrameError(Exception):
    pass

def point2sjtsk(point,frame,utm_zone,utm_frame,utm_local_frame, tf_buffer):
    sjtsk_coords = None

    if frame == "sjtsk":
        sjtsk_coords = point

    elif frame == "utm":
        if utm_zone is None:
            raise ValueError("point2sjtsk: utm_zone has not been set.")
        sjtsk_coords = coord_transform_data(point, get_utm_to_sjtsk_trans(utm_zone[2], utm_zone[:2]), dtype=np.float64)

    elif frame == "utm_local":
        try:
            utm_local_trans = tf_buffer.lookup_transform(utm_frame, utm_local_frame, rospy.Time())
        except:
            rospy.logwarn("point2sjtsk: Cannot obtain transform (utm, utm_local).")
            if utm_local_trans is not None:
                rospy.logwarn("point2sjtsk: Using last known transform.")
            else:
                rospy.logwarn("point2sjtsk: Returning None.")
                return None
            
        point[0] += self.utm_local_trans.transform.translation.x
        point[1] += self.utm_local_trans.transform.translation.y

        if self.utm_zone is None:
            raise ValueError("utm_zone has not been set.")
        
        sjtsk_coords = self.coord_transform_data(point, get_utm_to_sjtsk_trans(self.utm_zone[2], self.utm_zone[:2]), dtype=np.float64)

    elif frame == "wgs":
        sjtsk_coords = self.coord_transform_data(point, WGS_TO_SJTSK, dtype=np.float64)

    else:
        raise UnsupportedFrameError("Frame {} is not one of ('sjtsk','utm','utm_local','wgs').".format(frame))
    
    return sjtsk_coords