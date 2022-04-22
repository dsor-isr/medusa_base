pipeline {
    agent {
        docker {
            image 'harbor.dsor.isr.tecnico.ulisboa.pt/medusa/medusa_base_jenkins:latest'
            registryUrl 'https://harbor.dsor.isr.tecnico.ulisboa.pt'
            registryCredentialsId 'harbor-robot-token'
        }
    }
    stages {
        // Checkout stage to make sure the submodules are updated to the correct version
        stage('Checkout') {
            steps{
                echo 'Checkout..'
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
                        credentialsId: 'github_token', 
                        url: 'git@github.com:dsor-isr/medusa_base.git']]])
            }
        }
        // Build stage - compile the code
        stage('Build') {
            steps {
                echo 'Build..'
                dir('catkin_ws/src') {
                    sh '''#!/bin/bashs
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
