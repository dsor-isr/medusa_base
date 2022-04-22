pipeline {
    agent {
        docker {
            image 'harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:v0.0.3'
            registryUrl 'https://harbor.dsor.isr.tecnico.ulisboa.pt'
            registryCredentialsId 'harbor-robot-token'
            args '--entrypoint=""'
        }
    }
    environment {
        ROS_WORKSPACE = "${HOME}/catkin_ws"
    }
    // Clone the packages in the default catkin workspace
    options {
        checkoutToSubdirectory('${ROS_WORKSPACE}')
    }
    stages {
        // Build stage - compile the code
        stage('Build') {
            steps {
                echo 'Build..'
                dir(path: "${ROS_WORKSPACE}") {
                    sh '''#!/bin/bash
                    source /opt/ros/noetic/setup.bash
                    ls
                    catkin build --no-status'''
                }
            }
        }
        // Test stage - test the code
        stage('Test') {
            steps {
                echo 'Testing..'
            }
        }
        // Generate Doxygen documentation
        stage('Documentation') {
            when {tag "release-*"}
            steps{
                echo 'Generating Doxygen Documentation..'
            }
        }
    }
}
