#!/bin/bash
export PLANNING_GROUP="whole_body"
export BASE_LINK="0"
export EEF_LINK="12"
export FREE_INDEX="1"
export IKFAST_OUTPUT_PATH=`pwd`/ikfast61.cpp

openrave0.9.py --database inversekinematics --robot=/ikfast/robot/wrapper.xml --iktype=Transform6D --iktests=100
