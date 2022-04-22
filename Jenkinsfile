pipeline {
    agent {
        docker {
            image 'harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:v0.0.3'
            registryUrl 'https://harbor.dsor.isr.tecnico.ulisboa.pt'
            registryCredentialsId 'harbor-robot-token'
            args '--entrypoint=""'
            reuseNode false
        }
    }
    environment {
        ROS_WORKSPACE = "catkin_ws"
    }
    options {
        checkoutToSubdirectory("${ROS_WORKSPACE}/src")
    }
    // Move all the packages to the default catkin workspace
    stages {
        // Build stage - compile the code (using 12 threads)
        stage('Build') {
            steps {
                echo 'Build..'
                dir(path: "${ROS_WORKSPACE}") {
                    sh '''#!/bin/bash
                    source /opt/ros/noetic/setup.bash
                    catkin build --no-status -j12'''
                }
            }
        }
        // Test stage - test the code
        stage('Test') {
            steps {
                echo 'Testing..'
                dir(path: "${ROS_WORKSPACE}") {
                    sh '''#!/bin/bash
                    source /opt/ros/noetic/setup.bash
                    source ${ROS_WORKSPACE}/devel/setup.bash
                    catkin test --no-status -j12
                    '''
                }
            }
        }
        // Generate Doxygen documentation
        // only in release tags
        stage('Documentation') {
            when {tag "release-*"}
            steps{
                echo 'Generating Doxygen Documentation..'
            }
        }
    }
    // Cleanup the jenkins environment after running
    post {
        always {
            echo "Pipeline finished do cleanup"
            deleteDir()
        }
        success {
            echo "Release Success"
        }
        failure {
            echo "Release Failed"
        }
        cleanup {
            echo "Clean up in post work space"
            cleanWs()
        }
    }
}