#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/wiimote"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ubuntu/Desktop/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ubuntu/Desktop/catkin_ws/install/lib/python3/dist-packages:/home/ubuntu/Desktop/catkin_ws/build/wiimote/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ubuntu/Desktop/catkin_ws/build/wiimote" \
    "/usr/bin/python3" \
    "/home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/wiimote/setup.py" \
    egg_info --egg-base /home/ubuntu/Desktop/catkin_ws/build/wiimote \
    build --build-base "/home/ubuntu/Desktop/catkin_ws/build/wiimote" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ubuntu/Desktop/catkin_ws/install" --install-scripts="/home/ubuntu/Desktop/catkin_ws/install/bin"
