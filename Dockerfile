FROM ros:kinetic-ros-base-xenial

ENV PACKAGE example

RUN apt-get update && apt-get upgrade -y && apt-get install -y git python python-pip default-jdk-headless gradle

RUN pip install --upgrade pip

WORKDIR /srv/

RUN git clone https://github.com/eProsima/Fast-RTPS

WORKDIR /srv/Fast-RTPS

# eProsima Fast-RTPS v1.7.2
RUN git checkout a8691a40be6b8460b01edde36ad8563170a3a35a

RUN mkdir -p Fast-RTPS/build

WORKDIR /srv/Fast-RTPS/build

RUN cmake -DTHIRDPARTY=ON -DBUILD_JAVA=ON ..

RUN make

RUN make install

RUN ldconfig

RUN pip install catkin-tools

RUN mkdir -p /srv/${PACKAGE}

WORKDIR /srv/${PACKAGE}

COPY src /srv/${PACKAGE}/src

RUN echo "#!/usr/bin/env bash" > build.sh && \
    echo "cd /srv/${PACKAGE}" >> build.sh && \
    echo ". /opt/ros/kinetic/setup.bash" >> build.sh && \
    echo "catkin_make" >> build.sh && \
    echo "catkin_make install" >> build.sh && \
    echo "catkin_make tests" >> build.sh && \
    chmod +x build.sh

RUN echo "#!/usr/bin/env bash" > test.sh && \
    echo "cd /srv/${PACKAGE}" >> test.sh && \
    echo ". install/setup.sh" >> test.sh && \
    echo "catkin_make run_tests" >> test.sh && \
    echo "RETURNLEVEL=$?" >> test.sh && \
    echo "cp -frv build/test_results/${PACKAGE}/*.xml /srv/test_results/" >> test.sh && \
    echo "exit ${RETURNLEVEL}" >> test.sh && \
    chmod +x test.sh

RUN mkdir /srv/test_results

CMD tail -f /dev/null
