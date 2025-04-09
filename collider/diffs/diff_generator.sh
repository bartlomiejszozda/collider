# call it like this: ./diff_tmp `find ~/ardu_ws -name "iris_with_gimbal"`
echo $1
echo $2

git diff --no-index $2 $1 > ./new_diff
