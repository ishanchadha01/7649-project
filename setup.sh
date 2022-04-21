sourced=0
if [ -n "$ZSH_EVAL_CONTEXT" ]; then
  case $ZSH_EVAL_CONTEXT in *:file) sourced=1;; esac
elif [ -n "$BASH_VERSION" ]; then
  (return 0 2>/dev/null) && sourced=1
else # All other shells: examine $0 for known shell binary filenames
  # Detects `sh` and `dash`; add additional shell filenames as needed.
  echo "Please use bash or zsh for best experience with ROS"
fi
if [ "$sourced" != "1" ]; then
  echo "You must use source for setup.sh"
  usage
  exit # not return because "you can only return from a funtion or sourced script"
fi

source /usr/share/gazebo/setup.sh
source /opt/ros/noetic/setup.bash

if command -v conda >/dev/null; then
  echo "conda is installed, deactivating environment..."
  conda deactivate
fi

if [ -f "devel/setup.sh" ]; then
  echo "devel/setup.sh is present, sourcing it..."
  source devel/setup.sh
fi

if [ -f "install/setup.sh" ]; then
  echo "install/setup.sh is present, sourcing it..."
  source install/setup.sh
fi

# export ROS_DOMAIN_ID=69
