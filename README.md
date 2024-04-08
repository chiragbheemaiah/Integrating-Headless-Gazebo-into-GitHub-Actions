# Integrating-Headless-Gazebo-into-GitHub-Actions
[![CI Pipeline for ROS2 Turtlebot3](https://github.com/chiragbheemaiah/Integrating-Headless-Gazebo-into-GitHub-Actions/actions/workflows/main.yml/badge.svg)](https://github.com/chiragbheemaiah/Integrating-Headless-Gazebo-into-GitHub-Actions/actions/workflows/main.yml)

Software for Robotics - CSC591 Project by cpalang@ncsu.edu, ggopala4@ncsu.edu <br><br>
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
**Milestones:**<br>
![image](https://media.github.ncsu.edu/user/29852/files/10c28f83-f9dc-45e0-b0a7-5c105df9aa4f)


