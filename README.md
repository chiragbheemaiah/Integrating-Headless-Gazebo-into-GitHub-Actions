# Integrating-Headless-Gazebo-into-GitHub-Actions
[![CI Pipeline for ROS2 Turtlebot3](https://github.com/chiragbheemaiah/Integrating-Headless-Gazebo-into-GitHub-Actions/actions/workflows/main.yml/badge.svg)](https://github.com/chiragbheemaiah/Integrating-Headless-Gazebo-into-GitHub-Actions/actions/workflows/main.yml)

Software for Robotics - CSC591 Project by cpalang@ncsu.edu, ggopala4@ncsu.edu <br><br>
Platform - Turtlebot3 <br>
**Summary:**<br>
Integrating headless Gazebo simulations into the GitHub Actions pipeline for ROS2 development offers a promising solution to improve the efficiency of automated testing in robotic systems. The motivation behind this endeavor stems from the benefits of automated testing, such as eliminating human variability, speeding up bug detection, and promoting a more efficient development process through continuous integration. Operating Gazebo simulations in a headless mode, without a graphical user interface (GUI), leads to significant performance improvements by reducing computational overhead and accelerating the execution of tests [1]. This integration is designed to deliver prompt feedback on code changes, facilitating more frequent iterations and experimentation for developers.
<br>
Based on the references and related work, we propose a CI/CD solution for a given robotic system (TurtleBot or Clearpath Husky) incorporating headless Gazebo, a tool for robotic simulation, along with Github Actions, a continuous integration tool used extensively. Additionally, we plan to implement 1-2 core test cases that can be automated through the aforementioned CI/CD pipeline and also have secondary test cases that will be part of the stretch goals
<br><br>
**Motivation:**<br>
Automated testing provides efficiency, repeatability, and accuracy advantages by eliminating human variability, speeding up bug detection, and being cost-effective in the long run. It excels in handling large-scale tasks and enables a streamlined development process through continuous integration and delivery. Such an approach helps the Quality Assurance team since repeated tasks are often mundane and more prone to errors. Testers can focus more on higher-value activities such as exploratory testing, test planning, etc. 
In the context of ROS2 development, integrating headless Gazebo simulations with GitHub Actions aims to enhance automated testing for greater efficiency and speed. Operating without a GUI, headless Gazebo simulations offer faster performance, minimizing computational overhead and expediting test execution. This not only provides quick feedback on code changes but also boosts developer productivity by facilitating more frequent iterations and experimentation.
<br><br>
**Challenge:**<br>
Integrating headless Gazebo simulations into the GitHub Actions pipeline for ROS2 is quite a challenge. It involves getting the ROS2 framework for the chosen robotic system, headless Gazebo, and GitHub Actions to work smoothly together. This requires careful coordination and attention to compatibility and dependency setup. Another challenge would be evaluating simulation results in a headless environment. Without an interactive interface, it will be complicated to check the robot's behavior and identify issues. To overcome this, we need to create strong logging and analysis methods to accurately understand simulation outcomes. Dealing with these challenges is crucial for making sure that headless Gazebo simulations can seamlessly fit into the GitHub Actions pipeline for ROS2 development.
<br><br>
**Expected Approach:**
<br>
**Core Goals:**<br>
Robot Navigation Problem Experimentation - Could be any robotic system (turtlebot, clearpath husky) going from point A to point B. This can be useful for systems like consignment/food delivery robots.
Identify different test cases to perform functionality testing, simulation testing etc.
Design workflows combining headless Gazebo and GitHub Actions triggered based on git events such as pull, fork, among others.
<br>
**Stretch Goals:**<br>
Identify additional test cases by:
Changing the simulated environment.
Analyzing the performance of the bot by varying various parameters such as velocity, time, etc.
Compare and contrast the tools used to implement this pipeline.
<br><br>
**Milestones:** <br>
![image](https://media.github.ncsu.edu/user/29852/files/10c28f83-f9dc-45e0-b0a7-5c105df9aa4f)


**Contributions:** <br>
**Chirag**
1. Turtlebot3 setup and exploration
2. Defining ROS2 nodes for initial_pose and goal_pose, defining launch files for the integrated system
3. Dockerizing the test environment
4. Implementing the positive test case (Reachable navigation scenario)

**Girish**
1. Implementing the negative test case (Unreachable navigation scenario)
2. CI Pipeline Setup
3. Experimenting with simulation parameters for increasing efficiency of the pipeline

**Lessons Learnt and Challenges Faced**
1. The main challenge over the course of the project was that processes launched needed to be shut down in a certain order for the system to not crash when using a launch file. Especially processes relating to the Gazebo needed to be killed first followed by other processes, otherwise the entire system would just crash. We came across this problem while running automated tests wherein all the processes were shut down randomly once the test had concluded without waiting for the graceful termination of the process. We handled this challenge by integrating different sleep cycles for processes to shut down properly.
2. We first started out with using the `colcon test` framework for developing automated test suites. However, the lack of documentation of the `launch_test` framework which colcon test recommends affected our progress significantly due to the aforementioned reasons. Therefore, we migrated to the `pytest` framework wherein we waited for the launch process to complete execution gracefully and then proceeded with the tests.
3. During the initial iterations of our pipeline, we experimented with an approach where we configured the Ubuntu container provided by GitHub Actions with ROS2 Humble packages. We then checked out our repository and ran the automated testing suites. However, the build time for such a system was ~ 45 mins. Hence we dockerized the test environment and pulled it as an image inside the GitHub Actions container and ran our testing within this isolated container. This approach helped us reduce our build times to ~ 4 mins.
4. We used a **dev-container** (an extension of VS Code) which helps with seamless integration of VS Code with Docker containers which abstracted away the usual hassles of setting up ROS on a local system. Port forwarding mechanism and pre-configured when launching this setup helped to stream the visual output from the containers to the development environment through X11 forwarding. However, the current version of this extension has a bug where it increments the $DISPLAY system variable automatically. We solved this problem by initializing the $DISPLAY:=0 in the .bashrc script of the system. This particular approach helped greatly streamline our development process and is greatly recommended.
5. The integration of GitHub Actions workflow with our repository taught us the importance of following good software engineering practices. Pull Requests verified by these workflows greatly help an organization in reviewing code contributions and overall increasing the efficiency of the software developmental process.



