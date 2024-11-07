from parameters import RED, cv2, np
# output of subsystem


class MarkerStatus:
    def __init__(self):
        self.id = -1
        self.corners = []
        self.center_pt = (0, 0)
        self.top_pt = (0, 0)
        self.bottom_pt = (0, 0)
        self.left_pt = (0, 0)
        self.right_pt = (0, 0)
        self.h_angle = 0
        self.v_angle = 0
        self.m_angle = 0
        self.m_distance = 0
        self.height = 0
        self.width = 0
        self.rvecs = (0,0)
        self.tvecs = (0,0)
        self.trash = (0,0)
        self.R = [[0,0,0],[0,0,0],[0,0,0]]

    @classmethod
    def reset(cls):
        cls.id = -1
        cls.corners = []
        cls.center_pt = (0, 0)
        cls.top_pt = (0, 0)
        cls.bottom_pt = (0, 0)
        cls.left_pt = (0, 0)
        cls.right_pt = (0, 0)
        cls.h_angle = 0
        cls.v_angle = 0
        cls.m_angle = 0
        cls.m_distance = 0
        cls.height = 0
        cls.width = 0
        cls.rvecs =(0,0)
        cls.tvecs=(0,0)
        cls.trash=(0,0)
        cls.R = [[0,0,0],[0,0,0],[0,0,0]]
# subsystem


class SelectTargetMarker:
    @classmethod
    def setup(cls):
        MarkerStatus.reset()

    @classmethod
    def stop(cls):
        pass

    @classmethod
    def run(cls, frame, markers, drone_pos, offset=(0, 0)):

        cls.drone_pos = drone_pos
        cls.marker_status_list = []
        all_marker_data = cls._get_all_markers(markers)
        
        for id, corners in all_marker_data:
            marker = MarkerStatus()
            marker.id = id

            br, bl, tl, tr = corners[0], corners[1], corners[2], corners[3]
            marker.center_pt = cls._get_midpoint([br, bl, tl, tr])
            marker.left_pt = cls._get_midpoint([bl, tl])
            marker.right_pt = cls._get_midpoint([br, tr])
            marker.bottom_pt = cls._get_midpoint([br, bl])
            marker.top_pt = cls._get_midpoint([tl, tr])
            
            marker.height = cls._length_segment(marker.bottom_pt, marker.top_pt)
            marker.width = cls._length_segment(marker.left_pt, marker.right_pt)
            
            marker.h_angle = cls._angle_between(marker.left_pt, marker.right_pt)
            marker.v_angle = cls._angle_between(marker.top_pt, marker.bottom_pt, vertical=True)
            
            offset_x, offset_y = int(offset[0] * marker.width), int(offset[1] * marker.height)
            marker_pos = (marker.center_pt[0] + offset_x, marker.center_pt[1] + offset_y)
            marker.m_angle = cls._angle_between(drone_pos, marker_pos, vertical=True)
            marker.m_distance = cls._length_segment(drone_pos, marker_pos)
            
            cls.marker_status_list.append(marker)
            cls.draw(frame, marker)

        return cls.marker_status_list
    @staticmethod
    def _get_all_markers(markers):
        all_marker_data = []
        
        if markers.ids is not None:
            for i in range(len(markers.ids)):
                id = markers.ids[i][0]
                corners = markers.corners[i][0]
                all_marker_data.append((id, corners))

        return all_marker_data

    def _get_marker_with_min_id(markers):
        target_id = -1
        target_corners = []

        if markers.ids is None:
            return target_id, target_corners

        for i in range(len(markers.ids)):
            id = markers.ids[i][0]
            if id < target_id or target_id == -1:
                target_id = id
                target_corners = markers.corners[i][0]

        return target_id, target_corners

    @staticmethod
    def _get_midpoint(corners):
        # corners = [p1,p2,p3,p4] with pi = (xi, yi)
        xc = yc = 0
        n = len(corners)
        for x, y in corners:
            xc += x
            yc += y
        xc = int(xc/n)
        yc = int(yc/n)
        return (xc, yc)

    @staticmethod
    def _angle_between(p1, p2, vertical=False):
        dx = p1[0]-p2[0]
        dy = p1[1]-p2[1]
        if not vertical:  # angle betwenn Horizantal axis and segment (p1,p2)
            return np.arctan(-dy/(dx+0.000001))
        else:  # angle betwenn vertical axis and segment (p1,p2)
            return np.arctan(-dx/(dy+0.000001))

    @staticmethod
    def _length_segment(p1, p2):
        return int(np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2))
    
    @staticmethod
    def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash,R

    @classmethod
    def draw(cls, frame, marker):
        # Ensure marker has valid corners before drawing
        if marker.id == -1 or not marker.corners or len(marker.corners) != 4:
            return

        # Draw detected marker
        cv2.aruco.drawDetectedMarkers(frame, np.array([[marker.corners]]), np.array([
            [marker.id]]), borderColor=RED)

        # Draw lines for marker’s axes
        cv2.line(frame, marker.top_pt, marker.bottom_pt, (255, 0, 0), 2)
        cv2.line(frame, marker.left_pt, marker.right_pt, (255, 0, 0), 2)

        # Offset lines
        top_pt_with_offset = tuple(np.array(marker.top_pt) + np.array(cls.offset))
        bottom_pt_with_offset = tuple(np.array(marker.bottom_pt) + np.array(cls.offset))
        left_pt_with_offset = tuple(np.array(marker.left_pt) + np.array(cls.offset))
        right_pt_with_offset = tuple(np.array(marker.right_pt) + np.array(cls.offset))

        cv2.line(frame, top_pt_with_offset, bottom_pt_with_offset, (255, 0, 0), 2)
        cv2.line(frame, left_pt_with_offset, right_pt_with_offset, (255, 0, 0), 2)

        # Draw line between drone position and marker
        if cls.drone_pos[0] != 0:
            cv2.line(frame, cls.drone_pos, cls.marker_pos, (0, 0, 255), 2)