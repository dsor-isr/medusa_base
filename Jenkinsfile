pipeline {
    agent {
        docker {
            image 'harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:latest'
        }
    }
    stages {
        stage('Build') {
            steps {
                dir('catkin_ws') {
                    sh '''#!/bin/bash
                    git pull && git submodule update --init --recursive
                    source /opt/ros/noetic/setup.bash
                    catkin build --no-status
                '''
                }
            }
        }
    }
}