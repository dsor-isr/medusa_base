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
    options {
        checkoutToSubdirectory('${ROS_WORKSPACE}')
        checkout([
            $class: 'GitSCM', 
            branches: [[name: '*']], 
            doGenerateSubmoduleConfigurations: false, 
            extensions: [[
                $class: 'SubmoduleOption', 
                disableSubmodules: false, 
                parentCredentials: true, 
                recursiveSubmodules: true, 
                reference: '', 
                trackingSubmodules: false]], 
            submoduleCfg: [], 
            userRemoteConfigs: [[
                credentialsId: 'github_app_tokn', 
                url: 'git@github.com:dsor-isr/medusa_base.git']]])
    }
    stages {
        // Build stage - compile the code
        stage('Build') {
            steps {
                echo 'Build..'
                dir('catkin_ws/src') {
                    sh '''#!/bin/bash
                    source /opt/ros/noetic/setup.bash
                    catkin build --no-status
                '''
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
            steps{
                echo 'Generating Doxygen Documentation..'
            }
        }
    }
}
