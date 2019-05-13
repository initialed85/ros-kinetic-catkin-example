pipeline {
    agent any
    stages {
        stage('build') {
            steps {
                sh './build.sh force'
            }
        }
        stage('test') {
            steps {
                sh './test.sh'
            }
        }
    }
}