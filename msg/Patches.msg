# Message defining the set of patches from a range image

std_msgs/Header header

# centers and normals are vectors of size 3*N, so read with stride 3
# pixels are vectors of size 2*N, so read with stride 2
# sizes is the length in pixels of the side of the square region on the image
# weights is the eigenvalue along the normal when doing SDV to get the normal of patch
# labels indicate if a patch belongs to ground (0), or walls (1), or none (-1)
float32[] centers
float32[] normals
float32[] covars
float32[] weights
int32[] labels
int32[] sizes
int32[] pixels
