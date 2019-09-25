#!/bin/bash

BRANCH="master"
EXTEND="/opt/ros/kinetic"

while getopts ":r:d:e:b:" o; do
    case "${o}" in
        r)
            REMOTE=${OPTARG}
            ;;
        d)
            DIR=${OPTARG}
            ;;
        e)
            EXTEND=${OPTARG}
            ;;
        b)
            BRANCH=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done

WSPATH=${HOME}/${DIR}

GITDIR="${REMOTE##*/}"
GITDIR="${GITDIR//.git/}"
GITPATH="${WSPATH}/src/${GITDIR}"

if [ -z "$REMOTE" ] && [ $BRANCH==master ] ; then
    echo "Assuming git repo already exist. Building"
elif [ -d "$GITPATH/.git" ] ; then
    echo "Git repo already exist. Not cloning"
    cd ${GITPATH} && git checkout ${BRANCH}
elif ! [ -d "$WSPATH" ] ; then
    mkdir -p ${WSPATH}/src
    git clone ${REMOTE} ${WSPATH}/src/${GITDIR}
    cd ${GITPATH} && git checkout ${BRANCH}
else
    echo "Cannot find or clone repo"
fi


cd ${WSPATH} && catkin config --extend ${HOME}/${EXTEND}/devel && catkin build

SETUP=${WSPATH}/devel/setup.bash

chmod 755 ${SETUP}

grep -qxF "source ${SETUP}" ~/.bashrc || echo "source ${SETUP}" >> ~/.bashrc && source ~/.bashrc

exit 0