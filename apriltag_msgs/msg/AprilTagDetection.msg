string family
int32 id
int32 hamming
float64 size
float32 goodness
float32 decision_margin
geometry_msgs/PoseStamped pose
Point centre                    # centre in (x,y) pixel coordinates
Point[4] corners                # corners of tag ((x1,y1),(x2,y2),...)
float64[9] homography           # 3x3 row-major homography matrix
