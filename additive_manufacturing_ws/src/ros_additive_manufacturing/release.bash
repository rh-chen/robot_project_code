#!/bin/bash
if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit 1
fi

os=ubuntu
os_version=bionic
ros_distro=melodic
rosdep_yaml_name="ros_additive_manufacturing"
rosdep_list_name="31-ros-additive-manufacturing"

# Empty means no, anything else means yes
ignore_missing_info=""
generate_deb()
{
  echo -e "\033[34m--------------------------------------------"
  echo -e "--------------------------------------------"
  echo -e "--------------------------------------------\033[39m"
  underscore_name="${1//\-/\_}"
  cd $underscore_name
  bloom-generate rosdebian --os-name $os --os-version $os_version --ros-distro $ros_distro
  :
  if [ ! -z "$ignore_missing_info" ];
  then
    sed -i 's/dh_shlibdeps /dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info /g' debian/rules
  fi
  fakeroot debian/rules binary
  rm -rf debian/ obj-x86_64-linux-gnu/
  cd ..
  echo -e "\033[34m--------------------------------------------"
  echo -e "--------------------------------------------"
  echo -e "--------------------------------------------\033[39m"
}

check_deb_installed()
{
  pkg_name=ros-$ros_distro-$1
  pkg_ok=$(dpkg-query -W --showformat='${Status}\n' $pkg_name | grep "install ok installed")
  if [ -z "$pkg_ok" ];
  then
    echo "$1 is not installed, aborting!"
    exit 1
  fi
}

installed_debs=()
install_deb()
{
  deb_file_name=$(ls "ros-$ros_distro-$1"*.deb)
  installed_debs+=("ros-$ros_distro-$1")
  dpkg -i $deb_file_name
}

installed_deb_info ()
{
  echo -ne "\033[31mTo remove installed debian packages use:\033[39m\n" \
  "\033[33msudo dpkg -r "
  # Reverse order
  for ((i=${#installed_debs[@]}-1; i>=0; i--));
  do
    echo -ne "${installed_debs[$i]} "
  done
  echo -e "\033[39m"
}

rosdep_keys=()
append_rosdep_key()
{
  rosdep_keys+=("$1")
  echo "yaml file:///etc/ros/rosdep/sources.list.d/$rosdep_yaml_name.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/$rosdep_list_name.list >/dev/null
  keys=''
  for i in "${rosdep_keys[@]}"
  do
    underscore_name="${i//\-/\_}"
    keys+="$underscore_name:
    $os: [ros-$ros_distro-$i]\n"
  done
  echo -e "$keys" | sudo tee "/etc/ros/rosdep/sources.list.d/$rosdep_yaml_name.yaml" >/dev/null
}

clear_rosdep_keys()
{
  rm "/etc/ros/rosdep/sources.list.d/$rosdep_yaml_name.yaml"
  rm "/etc/ros/rosdep/sources.list.d/$rosdep_list_name.list"
  rosdep update
  rosdep fix-permissions
}

# Add rosdep rules we will need later when installing debian packages
append_rosdep_key ram-msgs
append_rosdep_key ram-utils
append_rosdep_key ram-modify-trajectory
append_rosdep_key ram-display
append_rosdep_key ram-post-processor
append_rosdep_key ram-path-planning
rosdep update

# Generate debians and install them
generate_deb ram_documentation
install_deb ram-documentation

generate_deb ram-msgs
install_deb ram-msgs

ignore_missing_info="yes" # VTK CPack debian misses some information
generate_deb ram-utils
ignore_missing_info=""
install_deb ram-utils

generate_deb ram-modify-trajectory
install_deb ram-modify-trajectory

generate_deb ram-display
install_deb ram-display

generate_deb ram-post-processor
install_deb ram-post-processor

ignore_missing_info="yes" # VTK CPack debian misses some information
generate_deb ram-path-planning
ignore_missing_info=""
install_deb ram-path-planning

ignore_missing_info="yes" # VTK CPack debian misses some information
generate_deb ram_qt_guis
ignore_missing_info=""
install_deb ram-qt-guis

clear_rosdep_keys
zip -r ros_additive_manufacturing *.deb install.bash
rm *.deb *.ddeb
installed_deb_info
