![](RackMultipart20211019-4-1oxe68t_html_e9eb75123a8c849e.png)

**CONTROL DESIGN AND HARDWARE**

**IMPLEMENTATION OF A MULTI-ROTOR SYSTEMS**

BY

**OBOH EDWARD**  **OSARETIN**

**ENG1503587**

SUPERVISED BY

**ENGR. J.A. IGIMOH**

**DEPARTMENT OF COMPUTER ENGINEERING**

**FACULTY OF ENGINEERING**

**UNIVERSITY OF BENIN**

**A THESIS SUBMITTED IN PARTIAL FULFILMENT OF THE REQUIREMENTS FOR THE AWARD OF THE DEGREE OF BACHELOR** OF ENGINEERING (B.ENG) IN COMPUTER ENGINEERING, UNIVERSITY OF BENIN, EDO STATE, **NIGERIA.**

**JULY 2021**

**CERTIFICATION**

This is to certify that **OBOH EDWARD OSARETIN** an undergraduate student in the Department of Computer Engineering, Faculty of Engineering, University of Benin, Edo state, with Matriculation number **ENG1503587** satisfactorily completed this work on his own as a partial fulfillment of the requirement for the award of **Bachelor Degree in Engineering (B.Eng) in Computer Engineering.**


**Engr. J.A Igimoh**
**(Supervisor)**

**Engr. U. Iruansi**
**(Head of Department)**

**DEDICATION**

This project is dedicated to God for His mercy and protection and for the knowledge he has enabled me to acquire.

**ACKNOWLEDGEMENT**

My sincere gratitude goes to God Almighty for giving me the moral, courage and enthusiasm to embark on this project. Project which has opened my eyes to the different technology advancements under the scope of study of the work.

I appreciate the efforts of my parents **Mr. and Mrs. Oboh** for bringing me up morally and academically. I must register my profound gratitude to my parent for their guide, moral and financial supports.

I wish to appreciate the University of Benin for giving me this great opportunity. I also wish to appreciate the Head of department, Computer Engineering, **Engr. Dr. U. Iruansi** and my course adviser Engr A. Obayawana.

It is pertinent at this juncture to appreciate the efforts of my project supervisor **Engr. J.A Igimoh** for his support and encouragement.

Ialsowishtoappreciateall mylecturerswhoseeffortshavecontributedtomy knowledgebasein Engineering. I want to say a big **Thank**** you**.

**ABSTRACT**

Unmanned aerial vehicles (UAVs) are being increasingly used today than ever before in both military and civil applications. The rapid advancement in miniature sensors, actuators and processors has led to the development of powerful autopilot systems which play a major role in UAVs control by making the flight safer and more efficient. These gave a rise to small sized, interestingly featured commercial UAVs, one of which is the quadcopter.

Firstly, the project started by developing a reference frame and a mathematical model for a Quadcopter system. Next, a flight orientation estimation was determined through an assortment of MEMS sensors such as an accelerometer and a gyroscope. Each sensor was individually addressed as to its strengths and weaknesses with regards to orientation estimation. Key constructs of the system include hardware and software specifications for a flight controller, a radio system, and &quot;sensorless&quot; brushless motor controllers. An algorithm was then used for the data fusion of these various sensors. The fused data was then fed into a control system that will efficiently stabilize the quadcopter.

At the end, the project work overviewed crucial concepts involved in achieving quadcopter flight such as orientation estimation and control system implementation. The project would present researchers with comprehensive hardware and software specifications for a quadcopter system. The primary application for the system would be for research with regards to the implementation of advance control techniques as well as data acquisition.

**TABLE OF CONTENT**

#

[**CONTROL DESIGN AND HARDWARE** 0](#_Toc76980687)

[**IMPLEMENTATION OF A MULTI-ROTOR SYSTEMS** 0](#_Toc76980688)

[**CERTIFICATION** 1](#_Toc76980689)

[**DEDICATION** 2](#_Toc76980690)

[**ACKNOWLEDGEMENT** 3](#_Toc76980691)

[**ABSTRACT** 4](#_Toc76980692)

[**TABLE OF CONTENT** 5](#_Toc76980693)

[**CHAPTER 1** 7](#_Toc76980694)

[**INTRODUCTION** 7](#_Toc76980695)

[**1.1**** History of Quadcopters:** 7](#_Toc76980696)

[**1.2 Problem Statement:** 8](#_Toc76980697)

[**1.3 Motivation and Objective:** 9](#_Toc76980698)

[**1.4 Scope of Studies:** 9](#_Toc76980699)

[**CHAPTER 2** 10](#_Toc76980700)

[**RELATED WORKS** 10](#_Toc76980701)

[**2.1 Previous Work** 10](#_Toc76980702)

[**2.2 Contributions** 11](#_Toc76980703)

[**2.3 Common Quadcopter Motor Types** 12](#_Toc76980704)

[**2.4 Quadcopter Configurations and Frame Design** 13](#_Toc76980705)

[**2.5 Quadcopter System Architecture** 15](#_Toc76980706)

[**CHAPTER 3** 17](#_Toc76980707)

[**METHODOLOGY** 17](#_Toc76980708)

[**3.1 Reference Frame and Control Objective** 18](#_Toc76980709)

[**3.1.1 Quadcopter Reference Frame** 18](#_Toc76980710)

[**3.1.2 Control Objective** 18](#_Toc76980711)

[**3.1.3 Reason for Mathematical Derivation** 19](#_Toc76980712)

[**3.2 Hardware Components** 20](#_Toc76980713)

[**3.3 IMU Sensor Implementation** 20](#_Toc76980714)

[**3.3.1 3 Axis Accelerometer** 21](#_Toc76980715)

[**3.3.2 3 Axis Gyroscope** 22](#_Toc76980716)

[**3.3.3 Sensor Fusion** 23](#_Toc76980717)

[**3.4 Control System Implementation** 24](#_Toc76980718)

[**3.4.1 PID Basics** 25](#_Toc76980719)

[**3.4.2 PID Control Software** 27](#_Toc76980720)

[**3.5 Software Architecture** 28](#_Toc76980721)

[**3.4.1 Software Flowchart Diagram** 28](#_Toc76980722)

[**3.6 Hardware Implementation** 29](#_Toc76980723)

[**3.4.1 Flight Controller Hardware** 29](#_Toc76980724)

[**3.4.2 Remote Controller Hardware** 31](#_Toc76980725)

[**3.4.3 Electronic Speed Controller Hardware** 33](#_Toc76980726)

[**3.4.4 Brushless Motor** 34](#_Toc76980727)

[**CHAPTER 4** 36](#_Toc76980728)

[**EXPECTED RESULTS** 36](#_Toc76980729)

[**4.1 Result and Outcome** 36](#_Toc76980730)

[**4.2 Bill of Engineering Measurement and Evaluation** 36](#_Toc76980731)

[**CHAPTER 5** 38](#_Toc76980732)

[**CONCLUSION** 38](#_Toc76980733)

[**5.1 Overview** 38](#_Toc76980734)

[**5.2 Future Work** 38](#_Toc76980735)

[**REFERENCES** 39](#_Toc76980736)

[**APPENDIX** 40](#_Toc76980737)

[**Flight Controller Arduino Code** 40](#_Toc76980738)

[**Radio Transmitter Arduino Code** 57](#_Toc76980739)

[**Radio Receiver Arduino Code** 60](#_Toc76980740)

[**ESC Calibration Arduino Code** 65](#_Toc76980741)

#
# **CHAPTER 1**

##
# **INTRODUCTION**

A quadcopter also known as a quadrotor is a multi-rotor unmanned aerial vehicle (UAV). Quadcopters falls in the category of vertical take-off and landing (VTOL) UAVs. A quadcopter has four rotors in square formation at the equal distance from the center of mass of the vehicle. Speed of the rotors is manipulated to perform different maneuvers, hovering, take-off and landing. Before GPS and internet, drones were only available for military use, but with continuous development in the UAV technology and exceptional growth rate in the previous ten years the drones have become very popular among the civil sector. With growth in popularity drones&#39; market was valued at 18.14 billion USD and it is expected to reach 52.30 billion USD by 2025 (Markets 2018.)

Advancement and introduction of an impressive technology in drones has developed new fields of applications for it. Today drones are being used in several areas for various purposes. Stated below are some of common areas of drones&#39; applications. (Fiaz and Mukarram 2018a; VBROADCAST LIMITED 2019.)

- Aerial photography
- Search and rescue
- Agriculture
- Shipping and delivery
- Engineering applications
- 3-D mapping
- Research and science
- Aerial surveillance
- Mineral exploration
- Military use, etc.

  1.
## **History of Quadcopters:**

The history of Quadcopters starts in the beginning of the 20th century. The first ever quadcopter built was &quot;Gyroplane n: 01&quot; in 1907. This quadcopter had many limitations. Its stabilization was achieved by control of people on the ground. During the 1920&#39;s, other quadcopters with much improved performance were built by engineers who targeted the vertical flight. In later years, the concept of Vertical Take-off and landing (VTOL) (add abbreviation) has become of a major interest to aerial researches and thus the study of quadcopter structure has evolved. For various applications, the need for Unmanned aerial vehicles (UAV) (add abbreviation) has arose and thus requirements for small sized and efficient vehicles are considered. Recently, small sized quadcopters have become a subject of UAVs due to their agile maneuverability and indoor and outdoor flight capabilities. Quadcopter nowadays are cooperated for the purpose of achieving many consumer tasks and applications. Open-source systems are now available to allow people developing their own quadcopters with light and inexpensive electronic materials.

## **1.2 Problem Statement:**

The project entails quadcopter control theory, inertial measurement unit (IMU) orientation fusion, and then provides a comprehensive software/hardware platform. To establish a reference frame, the project specifies a mathematical coordinate system and then uses this system to determine control goals. The remainder of the project focuses on how to meet the proposed control goals. An overview of existing IMU sensors is presented as well as their trade-offs with respect to quadcopter flight. An algorithm is then used to overcome existing sensor limitations by fusing IMU data to obtain orientation. Once known orientation is established, the project then focuses on specifying multi-rotor kinematics, software, and hardware involved in flight control.

The primary problem this project attempts to solve is in reducing barriers to entry for advance control techniques. When building a multi-rotor vehicle (drone), designers are faced with the choice of paying for a custom designed aerial vehicle, building their own vehicle from scratch, or sacrificing controllability for an inexpensive off-the-shelf system. While numerous inexpensive off-the-shelf multi-rotor platforms are available, they often consist of proprietary modules even when advertised as open-source. Common examples of these black-box modules are sensor-less brushless motor controllers, flight controllers, and radios. These modules are often proprietary and have limited hardware specifications (Clean Flight, &quot;Clean Flight&quot;, 2016), (Open Pilot, &quot;Open Pilot&quot;, 2016). Consequently, for a researcher, the control and the modifiability of these modules is limited.

## **1.3 Motivation and Objective:**

A main objective of this work is to provide researchers with a functional, fully specified, and stabilized quadcopter. This system will be specified from scratch hardware and software with the intent of eliminating as many black box components as possible. In addition, this flight system will have an emphasis on theoretical control as well as IMU data collection making it a prime candidate for future research.

## **1.4 Scope of Studies:**

The objective of this project is to utilize the existing material to understand dynamic equations and behavior of quadcopters. Depending on the dynamic equations of the quadcopter a Proportional, Integral and Derivative (PID) based control system will be designed and implemented to achieve control of the quadcopter. The designed controller will be able to control attitude of the vehicle (Roll, Pitch and Yaw). The final project work will explain the PID controllers tuning process and integration of the designed controller with real hardware in detail. The project is primarily focused on the PID controller, other control strategies would not be explained in this project. Altitude control and autonomous navigation are not part of this project, altitude and position of the vehicle in an inertial frame will be controlled by the pilot commands. Hardware board, Sensors, Electrical and Mechanical components will be selected, and their general working and compatibility to each other will be discussed for implementation of the controller on the actual system. However, components manufacturing process, materials and detailed working are not concerned with the purpose of the project.

#
# **CHAPTER 2**

##
# **RELATED WORKS**

## **2.1 Previous Work**

Admittedly, much work has been done in the area of advance control of multi-rotor systems and these works are too numerous for a comprehensive listing. Consequently, key examples will be provided that were be used as a reference for the development of this project. For example, Robert Mahony presents a comprehensive method for modeling, orientation, and control of a quadcopter with state space methods (R. Mahony, V. Kumar, and P. Corke, 2012). Another example of advance control is where researchers at University of Zurich implemented a quadcopter with a model predictive control were able to perform extreme acrobatic maneuvers (M. Mueller and D. Raffaello, 2013), (S. Lupashin, A. Schoellig, M. Sherback, and R. D&#39;Andrea, 2010). Numerous other control techniques have been applied to quadcopters as well such as PID, LQR, LQR-PID, and H∞ (L. Argentim, W. Contrimas, P. Santos, and R. Aguiar, 2013), (G. Raffo, M. Ortega, and F. Rubio, 2013). With regards to open-source multi-rotor systems, Open Pilot and Clean Flight are perhaps two of the most popular open software flight controller systems specifications (Clean Flight, &quot;Clean Flight&quot;, 2016), (Open Pilot, &quot;Open Pilot&quot;, 2016). These frameworks support a broad range of multi-rotor vehicles from tri-copters to octo-copters. With regards to open-source software and hardware systems, the Pixhawk and Sparky systems feature an open-source flight controller (Pixhawk, 2016), (Tau Labs, &quot;Sparky 2&quot;, 2016). While these systems feature some open hardware and software, they integrate with systems that are proprietary. In regards to indoor autonomous control, this is an active area of research for all types of remote vehicles. Mapping of unknown environments has been conducted with mutli-rotor vehicles utilizing lidar and employing the iterative closest point (ICP) algorithm (S. Winkvist, 2013), (Z. Zhang, 1994), (J. Zhang and S. Singh, 2014). However, in these cases the multi-rotor vehicle was operated by a human. A team at MIT achieved autonomous indoor control of an aerial vehicle by combining lidar data and IMU data with an extended Kalman filter as well as a Gaussian particle filter (A. Bry, A. Bachrach, and N. Roy, 2012). However, in this case the environment had been pre-mapped and pre-determined trajectories were used. Comprehensive simultaneous mapping and control is still an ongoing area of research.

In order to provide context for future chapters, this chapter introduces the basic inputs and outputs common to a quadcopter system. As the name quadcopter implies, a quadcopter is a multi-rotor aircraft with four propellers. Beyond the similarity of four propellers, there is significant design diversity. This design diversity includes but is not limited to motor type and frame design.

![](RackMultipart20211019-4-1oxe68t_html_41d19961d7fd580e.png)

Figure 2.1: Examples of quadcopter implementations

The quadcopters in Fig. 2.1 are referenced throughout this chapter as demonstrations of different types of design. A defining aspect of these quadcopters is the motor type that they utilize. Consequently, the next section will overview the common types of quadcopter motors.

## **2.2 Contributions**

This project develops a functional quadcopter system with a higher level of integration than most other open-source options. The key contributions of this flight system are as follows:

- Provides open-source software / hardware files for a functional stabilized auto-leveling flight controller
- Provides open-source software / hardware files for a remote-control system
- Offers practical design insights for other researchers attempting to construct their own aerial vehicles

## **2.3 Common Quadcopter Motor Types**

Low-cost commercial quadcopters generally use electric DC motors such as brushed and brushless permanent magnet motors. In contrast to gas motors, there exist small electric motors that are light-weight, low-cost, and of simple construction. These features make small electric motors ideal for low-cost commercial quadcopters. Among these electric motors, two common types of DC electric motors exist which are brushed and brushless motors. As the name implies, brushed DC motors are mechanically commutated with a brush, are powered by DC, and are explained in detail in (C. Hubert, 2002). The control simplification and cost of brushed DC motors makes them a popular choice for micro-size quadcopters such as the Turnigy Micro-X shown in Fig. 2.1(d). However, the brush which mechanically commutates the motor results in friction losses as well as limited motor life span. Consequently, larger quadcopters often use brushless permanent magnet DC motors which are electronically commutated. Brushless motors are also split into two common types which are sensored and sensor-less motors. Examples of these two motor types are illustrated in Fig. 2.2.

![](RackMultipart20211019-4-1oxe68t_html_1dbc7b7b64da4f95.png)

Figure 2.2: Examples of sensored(a) and sensor-less(b) brush-less permanent magnet motors

In order to electronically commutate a permanent magnet motor, the rotor&#39;s position must be known. This can be achieved by using Hall effect sensors or by using sensor-less driving techniques. Sensored motor operation simplifies driving complexity but results in heavier and more expensive motor as seen in Fig. 2.2. Instead of using sensors, sensor-less techniques such as back electro-motive force (BEMF) zero-cross detection and field-oriented control can be implemented (NXP, 2016). Sensor-less operation is desired since sensor-less motors have reduced weight, cost, and complexity. As a result of this, sensor-less brushless motors are common in quadcopters. However, in order to drive these motors, a DC to AC 3 phase sensor-less motor driver is needed. In terms of popular multi-rotor vernacular, these are commonly referred to as electronic speed controllers (ESC).

![](RackMultipart20211019-4-1oxe68t_html_a4a3cd72b7d8fa00.png)

Figure 2.3: Example of ESC

These ESCs represent an integral part of the quadcopter system architecture since their output controls the orientation of the quadcopter by varying the speed of the propellers.

## **2.4 Quadcopter Configurations and Frame Design**

Quadcopters have various configurations though the most common types are the X configuration, the H configuration, and the + configuration. These various configurations are illustrated in Fig. 2.4.

![](RackMultipart20211019-4-1oxe68t_html_5d9806561d3da515.png)

Figure 2.4: (a): X, (b): H, (c): + configuration quadcopters. In this figure, arrows represent propeller direction, wires represent rigid frame, and the center represents the quad&#39;s frame body

Each type of configuration offers advantages and disadvantages. The X configuration is the most commonly utilized motor configuration since it is simple to construct, ideal for a forward-facing camera, and is symmetrical. Quadcopters in Fig. 2.1(a,b,c) are using this type of configuration. A disadvantage of this configuration is an increase in control complexity. In contrast to the X configuration, the + configuration is the simplest to mathematically model and control. However, this configuration is least ideal for a forward-facing camera. Consequently, very few commercial drones are sold in this configuration and they generally only appear in research or DIY projects. Finally, the H configuration is sometimes built for mechanical convenience as seen in Fig. 2.1(d). In contrast, the DJI Inspire in Fig. 2.1(c) was designed as a H configuration quadcopter to achieve improved camera perspective (DJI, 2015). While this configuration can be ideal for forward facing cameras, it is also not symmetrical about its center. This lack of symmetry should be taken into account and should be considered when choosing which configuration to with. These three configurations including minor deviations represent the majority of quadcopter configurations in common use today.

## **2.5 Quadcopter System Architecture**

A step in presenting necessary background information for the quadcopter project is to briefly overview the electrical architecture. An example block diagram of the architecture unique to the most quadcopters is shown in Fig. 2.5.

![](RackMultipart20211019-4-1oxe68t_html_d46947c02f4df563.png)

Figure 2.5: Drone electrical architecture.

The underlying system for the quadcopter shown above is named the &quot;Marq Drone&quot; system. This system consists of a flight controller, a radio, and ESCs. Core components of the flight controller include a micro-controller unit (MCU), a radio, and an inertial measurement unit (IMU). The flight controller communicates with the ESCs through pulse position modulation (PPM). More information about PPM can be found here (G Lazaridis, 2011).

In addition to communicating with ESCs, the flight controller shown above communicates with a lidar sensor through a universal asynchronous receive and transmit (UART) interface. The build of the quadcopter did not include a lidar sensor and UART interface because I decided to make use of a cheap low-end microcontroller as the MCU in my build. A lidar sensor could be seamlessly interfaced with our choice of microcontroller but as the number of sensors to be interfaced increases, the response time of the system will increase, hence the exemption of the lidar sensor. To communicate with devices external to the quadcopter, a USB interface can be used for data acquisition or programming. Another communication method is through a wireless 2.4GHz frequency shift keyed (FSK) interface for receiving flight commands from the radio controller and transceiving flight data. Now that all the components involved have been introduced, the I/O of the system can be identified. For the flight controller itself, feedback inputs are from the IMU sensor. The outputs of the system are the four individual PPM signals that are sent to the ESCs. In the project work, a control scheme was derived that uses these I/O to achieve stable flight.

#
# **CHAPTER 3**

##
# **METHODOLOGY**

This chapter overviews the electronic hardware design of the quadcopter. These include the radio system, flight controller, and motor controllers. An example picture of a quadcopter is shown below

![](RackMultipart20211019-4-1oxe68t_html_a093d7c7f3e04a8.jpg)

These pieces of hardware in the diagram above were chosen in order to help understand the nature of the underlying system of a quadcopter as well to be able to implement a more flexible architecture.

## **3.1 Reference Frame and Control Objective**

A reference coordinate system for the quadcopter will be presented in addition to control goals. Like other multi-rotor vehicles, the primary control objective is to keep the quadcopter&#39;s orientation controlled and its altitude above ground non zero. With this in mind, a commonly used reference coordinate system will be presented to explicitly define this control objective.

### **3.1.1 Quadcopter Reference Frame**

![](RackMultipart20211019-4-1oxe68t_html_ddcc148fdcdd0d92.png)

The figure above demonstrates a reference frame for quadcopters. The reference frame uses Cartesian x,y,z system of coordinates for position and uses Euler angles pitch (θ), roll (φ), and yaw (ψ) to denote orientation. The origin the axes represents the center of the quadcopter as well as the center of mass.

### **3.1.2 Control Objective**

A primary control objective for the quadcopter is to be able to control orientation. With this in mind, the first set of control goals can be summarized in equation 3.1 where θc, φc, φc represent a user commanded rotation, where ˙θ, φ,˙ ψ˙ represent the rotational velocities of the quadcopter, and where Tc represents the user commanded throttle.

˙θ = 0 φ˙ = 0 ψ˙ = 0

θ → θc φ → φc ψ → ψc, T → Tc

(3.1)

These control parameters, θc, φc, ψc, Tc, allow a user to maneuver the quadcopter to anywhere in three-dimensional space. Consequently, many commercial systems give users these four degrees of freedom to operate a quadcopter. However, the user must also act as a control system in order to regulate the quadcopter&#39;s height to keep it above the ground. To achieve this, the user must observe the quadcopter&#39;s height and constantly adjust the throttle, Tc, such that the height with respect to ground, z, is roughly constant. Also, if a user desires a quadcopter to stay at a fixed position in space, they must also observe the x, y position of the quadcopter and adjust θ, φ accordingly. While these parameters allow a user to control a quadcopter, these four degrees of freedom are not enough for stable autonomous flight. Without a user to observe and control the quadcopters x, y, z position with respect to the room, stable flight is not possible. Therefore, as a secondary control objective, equation 3.2 demonstrates the control needed for autonomous flight where xc, yc, zc represents a user desired position.

x˙ = 0, y˙ = 0, z˙ = 0

x → xc y → yc z → zc

(3.2)

To meet the autonomous control objectives in equation 3.2, additional sensors are needed such as lidar, GPS, and sonar. These methods will be introduced later in this thesis.

### **3.1.3 Reason for Mathematical Derivation**

A mathematical derivation of a multi-rotor frame provides a basis for update laws and helps a designer gain intuition regarding the system. In simple cases, these update laws can be derived from visual inspection of the airframe.

However, this method becomes less effective as motor count is increased, as asymmetries are introduced to the air frame, and as motor angles are changed. Consequently, the incentive of the following sections is to translate input of motor speed to angular velocities (˙θ, φ, ψ˙) such that the control system can drive pitch (θ), roll (φ), and yaw (ψ) to their desired values.

## **3.2 Hardware Components**

The components were selected considering the performance and compatibility with other selected components. The most suitable components were selected considering the application and budget of the project to build the actual model.

| Frame | DJI F450 Quadcopter Frame |
| --- | --- |
| Propellers/Rotors | 10x4.5 Propellers (4) |
| Motors | A2212 BLDC motors |
| ESCs | Hobby King 30A Brushless ESC |
| Battery | 11.1V Li-po Battery |
| Transmitter | NRF24L10 PA (power amplifier) LNA (low noise amplifier) |
| Receiver | NRF24L10 |
| Inertial Measurement Unit (IMU) | MPU 6050 6dof IMU |
| Arduino Boards | Arduino UNO (1), Arduino Nano (2) |

## **3.3 IMU Sensor Implementation**

In order to meet the control goals, estimation of the quadcopter&#39;s orientation is required. With modern advancements in electronics, determining orientation can be done inexpensively, efficiently, and quickly with micro electrical mechanical system (MEMS) based sensors. Unfortunately, currently, there is no single affordable MEMS sensor that directly measures θ, φ, ψ. Consequently, the combination of multiple MEMS sensors is required in order to accurately estimate orientation. This section overviews the operation of relevant MEMS sensors as well as their strengths and weaknesses with respect to multi-rotor aircraft. The sensor used for Inertial Measurement Unit (IMU) is the MPU 6050 6dof IMU.

### **3.3.1 3 Axis Accelerometer**

3 axis accelerometers are devices that are designed to measure Cartesian ¯x, y, ¯ z¯ accelerations. It follows that the output of the device is an acceleration vector denoted as A = [Ax, Ay, Az].

![](RackMultipart20211019-4-1oxe68t_html_9878fcebdcc7a404.png)

Spring Accelerometer

![](RackMultipart20211019-4-1oxe68t_html_d76a6b98c148e2e8.png)

Screenshot of code showing Accelerometer sensor reading and calculation

### **3.3.2 3 Axis Gyroscope**

In contrast to the accelerometers, gyroscopes measure rotation. For decades, gyroscopes have been used as an integral part in naval and aerospace navigational systems. The first gyroscopes were generally spinning masses held in a gimbal frame. These frames allowed the mass to spin freely and maintain its axis of rotation regardless of the orientation of the external frame. Due to conservation of angular momentum, the spinning mass would resist any changes to its rotation. The structure is illustrated below

![](RackMultipart20211019-4-1oxe68t_html_c36ffb1544d3cbba.png)

Classical Gyroscope or Gyrostat

To electronically measure changes in rotation using a classical gyroscope, electrical potentiometers are connected to the gimbal frames. While these devices are capable of a large degree of accuracy, they are large, heavy, and expensive.

![](RackMultipart20211019-4-1oxe68t_html_4800366ff9757341.png)

Screenshot of code showing Gyroscope sensor reading and calculation

### **3.3.3 Sensor Fusion**

The process of combining data from multiple sensors and coming up with a collective estimate is commonly called sensor fusion. For orientation, there are multiple different sensor fusion algorithms such as gradient descent and Kalman filtering. However, due to the high frequency vibrations, large system accelerations, low computational power available, and fast control loop update requirements, not all algorithms are ideal for quadcopter flight use. With this in mind, a simple complementary filter was selected. This work is shown in the figure below where subscript k represents the sample number.

![](RackMultipart20211019-4-1oxe68t_html_47134b51a6815f9f.png)

Block Diagram showing Sensor Fusion

A block diagram representation of a basic complementary filter. Where Ax,y,z is the raw accelerometer data and G˙θ,φ,˙ ψ˙ is the raw gyroscope data. The weakness addressed by this filter are; while gyroscopes have good dynamic response and noise immunity, they have long term drift. In contrast, accelerometers have poor dynamic response but are not susceptible to drift in the same manner. Consequently, a high pass filter is used on the gyroscope data and a low pass filter is used on the accelerometer data. However, before the data is combined, it is important to verify validity of the accelerometer data. Also, before combining the gyroscope and accelerometer data, the accelerometer&#39;s data must be converted to orientation angles using equation 3.3 below

![](RackMultipart20211019-4-1oxe68t_html_5cfad8330998a58.png)

(3.3)

![](RackMultipart20211019-4-1oxe68t_html_430813f2fde5bd97.png)

Screenshot of code showing Sensor fusion between Accelerometer and Gyroscope

## **3.4 Control System Implementation**

This section will overview the control system used to achieve stable flight and meet the control objectives. This control system was needed in order to handle real world disturbances and to account for unknown offsets. In order to drive the quadcopter&#39;s orientation (θ, φ, ψ) to desired values, a series of proportional integral derivative (PID) controllers were implemented.

### **3.4.1 PID Basics**

A PID controller is a control loop that updates based upon the error observed between a desired output and the measured output. This control loop&#39;s response is characterized by coefficients Kp, Ki , Kd and is shown in equation 3.3 where e represents error and u(t) represents the PID output.

u(t) = Kp · e(t) + Ki · Z t 0 e(τ )dτ + Kd de(t) dt

(3.3)

The magnitude of coefficients Kp, Ki , Kd determines how responsive the controller is to proportional, integral, and derivative errors respectively. Control loops such as PID act on systems to stabilize them if needed. In addition, a PID loop can improve the transient response of a system. Systems themselves in controls context are often referred to as a &quot;plant&quot;. In the case of this project, the plant is the quadcopter system. This plant along with control is visualized in the figure below, where u(t) is the input to the plant, where r(t) is the desired set point of the plant, and where e(t) is the error between the output of the plant and the desired set point.

![](RackMultipart20211019-4-1oxe68t_html_90b11c705b79b275.png)

Block Diagram of PID in Control Loop

The input to the plant, u(t), represents the speed commanded to each of the quadcopter&#39;s motors. For simplicity, u(t) will be redefined to vector T where T = [T0, T1, T2, T3]. T0, T1, T2, T3 represent the four individual motor thrust vectors. r(t), the desired quadcopter outputs, is the user commanded orientation angles θc, φc, ψc as well as throttle Tc. Recall that these outputs were previously defined in equation 3.1. Note that this system is linearized at a current point in time and consequently assumes that superposition is upheld. It follows that the input to the system, u(t), is calculated as a summation of the individual control components. This control concept is shown in the figure below and the calculations for each block are demonstrated in section 3.4.2.

![](RackMultipart20211019-4-1oxe68t_html_28564f8431c8f5d0.png)

Block Diagram of PID Error Fusion

With the input and output relationship established, the following section overviews the PID code developed for the system as well as how T is updated such that the control goals are achieved.

### **3.4.2 PID Control Software**

![](RackMultipart20211019-4-1oxe68t_html_c610dd86ef4b87f6.png)

![](RackMultipart20211019-4-1oxe68t_html_65bd0b9586902399.png)

Screenshot showing PID Controller implementation in code

## **3.5 Software Architecture**

The control loop along with the IMU code shown in the flowchart diagram below account for the majority of flight critical code running on the quadcopter. To keep control timing and IMU collection constant, the software was implemented in three separate threads. These threads primarily manage IMU filter updates, control system updates, and radio link updates. For this quadcopter software implementation, IMU data was received on a consistent interrupt basis. Gyro and accelerometer readings were updated and filtered at 500 Hz. The magnetometer was updated at 100 Hz which is the maximum for the selected device. These updates were performed based on an interrupt to give the IMU filter constant timing characteristics. Likewise, the control system was also triggered through interrupts. The control system was executed every 200 Hz and used the most up to date data from both the controller and the IMU filter to perform its calculations. Less timing critical tasks were then executed in a super loop such as checking for new radio data and other non-flight related features.

### **3.4.1 Software Flowchart Diagram**

![](RackMultipart20211019-4-1oxe68t_html_24f40730cb97c726.png)

Flowchart Diagram of Software Loop

## **3.6 Hardware Implementation**

This section overviews the electronic hardware design of the quadcopter. Electronic hardware components of this system include the radio system, flight controller, and motor controllers. A picture of the quadcopter is shown below

![Picture 12](RackMultipart20211019-4-1oxe68t_html_916e135750721814.gif)

Image of Quadcopter

These pieces of hardware were designed to help understand the nature of the underlying system as well to be able to implement a more flexible architecture. Hardware involved in this project were selected separately from different manufacturers and are compatible with one another. The following sections overview the hardware design for the quadcopter.

### **3.4.1 Flight Controller Hardware**

The flight controller consists of a MCU (an Arduino UNO is used here), a IMU and Radio receiver. The flight controller is responsible for stabilizing the quadcopter and executing user control. To enable ease of use and easy implementation of the control software on cheap, easy to find hardware, the Arduino UNO was selected as the MCU. Below is a block diagram showing the connection between all elements which make up the flight control

![](RackMultipart20211019-4-1oxe68t_html_f53e99e5de2c9de3.jpg)

Block Diagram Showing Components in Flight Control Hardware

The ArduinoUno is a microcontroller board based on the ATmega328P. It has 14 digital input/output pins (of which 6 can be used as PWM outputs), 6 analog inputs, a 16 MHz quartz crystal, a USB connection, a power jack, an ICSP header and a reset button. It contains everything needed to support the microcontroller; We simply connect it to a computer with a USB cable or power it with an AC-to-DC adapter or battery to get it started.

With regards to the IMU, the MPU-6050 by Adafruit was selected since it is a compact integrated solution with a 3-axis accelerometer and a gyroscope. Another advantage of the MPU-6050 is that it had already been flight tested in another multi-rotor platform.

The radio receiver used here is the nRF24L10 from Nordic Semiconductor. The nRF24L10 is a wireless transceiver module, meaning each module can both send as well as receive data. They operate in the frequency of 2.4GHz, which falls under the ISM band and hence it is legal to use in almost all countries for engineering applications. The modules when operated efficiently can cover a distance of 100 meters (200 feet) which makes it a great choice for all wireless remote-controlled projects. The module operates at 3.3V hence can be easily used with 3.2V systems or 5V systems. Each module has an address range of 125 and each module can communicate with 6 other modules hence it is possible to have multiple wireless units communicating with each other in aparticular area. Hence mesh networks or other types of networks are possible using this module. Below is a photo showing the flight controller implementation

![](RackMultipart20211019-4-1oxe68t_html_2ae838b9a27d840.jpg)

Image of Flight Controller Hardware

### **3.4.2 Remote Controller Hardware**

The remote controller hardware consists of two joysticks, two toggle switches, two potentiometer switches, a micro-controller (am Arduino Nano in this case) with a USB interface, a radio receiver (nRF24L10 low noise – power amplifier), 3.3v voltage regulator and a 9v battery. A system level block diagram for the remote is shown in the figure below. This diagram illustrates all the major hardware blocks utilized by the controller. Further information can be derived from the source code as well as the schematic.

For the joysticks, two Gimbals were selected. These joysticks were selected since they were primarily designed for remote flight control and allow for easy integration with an Arduino. To simplify the mechanical design, these joysticks mount directly to the Veroboard which avoids requiring an injection molded or 3D printed enclosure. With regards to the microcontroller, an Arduino Nano was selected due to its low cost, availability, and USB interface. The radio transmitter used here is the same as that used in the flight controller which is the nRF24L10 except its variant is built for low noise and power amplification. Toggle switches were added for flight commands such as emergency cutoff and for extra commands to be added later. Two potentiometer switches were also added for alternative throttle control, LEDs were also added to indicate power supply to the board. Finally, for remote portability, a 1 cell 9V battery was used such that the remote could be operated independently of a computer. In addition, a 3.3v voltage regulator is also present in order to step down the 9v from the battery to 3.3v, suitable for the radio receiver. A picture of the assembled controller is illustrated below.

![](RackMultipart20211019-4-1oxe68t_html_2ebe5458c7cc2492.jpg)

Image of Drone Remote Controller

![](RackMultipart20211019-4-1oxe68t_html_e1e1537045d0d783.jpg)

Block Diagram of Remote Controller

### **3.4.3 Electronic Speed Controller Hardware**

The brushless sensorless motor controllers, or ESCs, are made up primarily of a microcontroller, gate drivers, and high-power FETs. The MCU selected was the Cortex M0+ KL25Z128 Kinetis by Freescale. The gate driver selected was the ADP3110 by ON Semiconductor since it supported dual high side low side NFET driving, supported 3.3V logic signals, is low cost, and has an easy to solder package. The primary switching FET selected was the PSMN011 by NXP since it supports a max drain current of 61A, has an RDSON of 10 mΩ, and since it utilizes a small QFN package. This ESC was designed to be compact and have capacity to drive at least 10A of current. To achieve this, double sided population was required.

![](RackMultipart20211019-4-1oxe68t_html_67622bb6c383dd9f.jpg)

Image of Electronic Speed Controller

### **3.4.4 Brushless Motor**

Brushless motors are synchronous 3 phase permanent magnet motors. Like a DC brushed motor, brushless motor consists of a stator which contains coils and a rotor which contains permanent magnets. If the coils are energized correctly, the coils create a rotational torque that acts on the magnets on the rotor. Since the magnets are adhered to rotor, this rotational torque causes the motor shaft to spin. However, if the coils on the stator are not energized correctly with respect to the polarity and position of the magnets, rotational torque is not generated. Consequently, knowledge of the rotors position is required in order to properly drive a DC motor. For brushed motors, a mechanical device called a brush is used such that stator coils are always properly energized with respect to the rotor. However, with a brushless motor, this mechanical linkage from stator to the rotor is removed in order to reduce friction and extend motor lifetime. Without the mechanical linkage between the rotor and the stator, external sensing methods of the rotor are required. Hall effect sensor can be used in order to detect the rotors position. However, hall effect sensors are expensive and require additional wires as well as mechanical protection. Another method of driving brushless motors is with sensor-less techniques such as back electromotive force (BEMF) zero cross detection.

![](RackMultipart20211019-4-1oxe68t_html_5103c62bc6d54a9f.jpg)

Image of Brushless DC Motor

#
# **CHAPTER 4**

##
# **EXPECTED RESULTS**

## **4.1 Result and Outcome**

For safety and verification purposes, experimental setup is designed to examine the controller response before applying it to a real flight test. An experimental testbench was used. On the test bench, the quadcopter is held firmly from the top and the bottom allowing only rotational motion, i.e., three degrees of freedom. Various reference signals are sent via RC to explore the system reference tracking.

At the end of testing, as expected, the multirotor system (Quadcopter in this case) was controlled by the electronic Remote controller using RF signal. The flight controller was able to control roll, pitch, yaw, altitude and motion in x or y direction. As expected, the multirotor system balanced itself using commands in the control software written to its microcontroller, when there were no changes to inputs sent from the remote controller.

## **4.2 Bill of Engineering Measurement and Evaluation**

| **Component** | **Specification** | **Number** | **Cost** |
| --- | --- | --- | --- |
| Frame | DJI F450 Quadcopter Frame | 1 | 8000 |
| Propellers/Rotors | 10x4.5 Propellers | 4 | 4 x 1500 |
| Motors | A2212 BLDC motors | 4 | 4 x 3500 |
| ESCs | Hobby King 30A Brushless ESC | 4 | 4 x 3000 |
| Battery | 11.1 V Li-po Battery | 1 | 8000 |
| Transmitter | NRF24L10 PA(power amplifier) LNA(low noise amplifier) | 1 | 3500 |
| Receiver | NRF24L10 | 1 | 3000 |
| Inertial Measurement Unit (IMU) | MPU 6050 6dof IMU | 1 | 2400 |
| Arduino UNO | Arduino UNO Artmega MCU | 1 | 8000 |
| Arduino Nano | Arduino Nano, Atmega MCU | 2 | 4200 |
| Vero Board | Dotted Copper Vero Board | 2 | 2 x 400 |
| Joystick | 2 Axis Joystick Potentiometer | 2 | 2 x 3500 |
| Other Components for soldering, connections, testing and binding
 | Switches, LEDs, Potentiometers, header connectors, connecting wires, Glue, solder, Battery cap | - | 3500 |
| Shipping and Delivery fee | - | - | 7000 |
|
 | **TOTAL** |
 | 87,400 |

#
# **CHAPTER 5**

##
# **CONCLUSION**

## **5.1 Overview**

The main purpose of this project was to develop both the hardware and software for a quadcopter system. We developed a control update laws as well as control objectives, we developed a 6 degrees of freedom fusion filter to provide accurate estimations of orientation, we demonstration a control system to achieve stable auto leveling flight using a series of PID controllers. We then overviewed the RF hardware involved in creating the transcieving link between the remote and the quadcopter. We specified other hardware involved in the system such as the flight controller and electronic speed controllers. Finally, the conclusion demonstrated flight results and performance of the overall system. With regards to contributions, this project provides reference source files and multi-rotor design insights. Specifically, this project provides open-source software / hardware files for a functional stabilized auto-leveling flight controller. It also provides open-source software / hardware files for a remote controller. It offers design insights into achieving multi-rotor flight with a 6 degree of freedom orientation fusion algorithm as well as a PID control system. Finally,

## **5.2 Future Work**

This section describes potential follow-up work to the current Drone Design. Currently, the IMU filter operates on Euler angles when ideally it should be based upon quaternions to avoid singularities. In addition, quaternions also allow for true linear spherical interpolation where as low pass filtering Euler angles does not produce a linear response. With regards to the control system, the first thing that would be improved is the PID control system performance. The current balancing action is somewhat noisy and the yaw control is lacking. In addition to PID control, other control techniques could be implemented such as linear quadratic regulator (LQR) as well as model predictive control (MPC). Another improvement would be to integrate the designed electronic speed controllers into quadcopter system. A final improvement would be the addition of an onboard GPS sensor and a Lidar sensor to facilitate autonomous navigation in both indoor and outdoor environments; addition of a camera to enable the system to be used for aerial surveillance or for gathering data.

#
# **REFERENCES**

A. Bry, A. Bachrach, and N. Roy (2012) &quot;State estimation for aggressive flight in gps-denied

environments using onboard sensing&quot;, in IEEE International Conference on Robotics and Automation, pp. 19–25

C. Hubert (2002), Electric Machines, 2nd edition.

Clean Flight, &quot;Clean Flight&quot; (2016), [Online]. Available at:

[http://cleanflight.com](http://cleanflight.com/)

DJI (2015), &quot;Inspire 1&quot;, [Online]. Available at:

[http://www.dji.com/product/inspire-1](http://www.dji.com/product/inspire-1)

G Lazaridis (2011), &quot;Pulse Position Modulation and Differential PPM&quot;, [Online]. Available at:

[http://www.pcbheaven.com/wikipages/Pulse\_Position\_Modulation/](http://www.pcbheaven.com/wikipages/Pulse_Position_Modulation/)

G. Raffo, M. Ortega, and F. Rubio (2013) &quot;An integral predictive/nonlinear h infinity control structure

for a quadrotor helicopter&quot;, in Automatica, pp. 1–7.

J. Zhang and S. Singh (2014), &quot;Loam: Lidar odometry and mapping in real-time&quot;, in Robotics: Science

and Systems Conference.

L. Argentim, W. Contrimas, P. Santos, and R. Aguiar (2013) &quot;Lqr and lqr-pid on a quadcopter platform&quot;,

in IEEE Int.Conference on Electronics, Informatics and Vision.

M. Mueller and D. Raffaello (2013) &quot;A model predictive controller for quadrocopter state interception&quot;,

in ControlConference (ECC), European. IEEE, pp. 1383–1389.

NXP (2016), &quot;3-Phase BLDC Motor Control with Sensorless Back EMF Zero Crossing Detection Using

56F80x&quot;, [Online]. Available: [https://cache.freescale.com/files/product/doc/AN1914.pdf](https://cache.freescale.com/files/product/doc/AN1914.pdf)

NXP (2016), &quot;Sensorless PMSM Field-Oriented Control&quot;, [Online]. Available at:

[http://www.nxp.com/doc/DRM148](http://www.nxp.com/doc/DRM148).

Open Pilot, &quot;Open Pilot&quot; (2016), [Online]. Available at:

[https://www.openpilot.org](https://www.openpilot.org/)

Pixhawk (2016), &quot;PIXHAWK is the all-in-one unit, combining FMU and IO into a single package&quot;,

[Online]. Available at: https://pixhawk.org/.

R. Mahony, V. Kumar, and P. Corke (2012) &quot;Multirotor aerial vehicles: Modeling, estimation, and

control of quadrotor&quot;, IEEE Robotics AutomationMagazine, vol. 19, no. 3, pp. 20–32.

S. Lupashin, A. Schoellig, M. Sherback, and R. D&#39;Andrea (2010) &quot;A simple learning strategy for high

speed quadrocopter multi-flips&quot;, in Robotics and Automation (ICRA), IEEE International

Conference on, pp. 1642–1648.

S. Winkvist (2013), &quot;Low computational SLAM for an autonomous indoor aerial inspection vehicle&quot;,

Master&#39;s thesis, University of Warwick, the Netherlands

Tau Labs, &quot;Sparky 2&quot; (2016), [Online]. Available at:

[https://github.com/TauLabs/TauLabs/wiki/Sparky2](https://github.com/TauLabs/TauLabs/wiki/Sparky2)

Z. Zhang (1994), &quot;Iterative point matching for registration of free-form curves and surfaces&quot;, in Int.

Journal of Computer Vision. pp. 19–25, IEEE.

#
# **APPENDIX**

## **Flight Controller Arduino Code**

/\*

\* Arduino pin | MPU6050

\* 5V | Vcc

\* GND | GND

\* A4 | SDA

\* A5 | SCL

\*

\* F\_ Left\_\_motor | D4

\* F\_ Right\_\_motor| D7

\* B\_Left\_\_motor | D5

\* B\_Right\_\_motor | D6

\*/

#include \&lt;Wire.h\&gt;

#include \&lt;Servo.h\&gt;

Servo L\_F\_prop;

Servo L\_B\_prop;

Servo R\_F\_prop;

Servo R\_B\_prop;

//We create variables for the time width values of each PWM input signal

unsigned long counter\_1, counter\_2, counter\_3, counter\_4, current\_count;

//We create 4 variables to stopre the previous value of the input signal (if LOW or HIGH)

byte last\_CH1\_state, last\_CH2\_state, last\_CH3\_state, last\_CH4\_state;

//To store the 1000us to 2000us value we create variables and store each channel

int input\_YAW; //In my case channel 4 of the receiver and pin D12 of arduino

int input\_PITCH; //In my case channel 2 of the receiver and pin D9 of arduino

int input\_ROLL; //In my case channel 1 of the receiver and pin D8 of arduino

int input\_THROTTLE; //In my case channel 3 of the receiver and pin D10 of arduino

/\*MPU-6050 gives you 16 bits data so you have to create some float constants

\*to store the data for accelerations and gyro\*/

//Gyro Variables

float elapsedTime, time, timePrev; //Variables for time control

int gyro\_error=0; //We use this variable to only calculate once the gyro data error

float Gyr\_rawX, Gyr\_rawY, Gyr\_rawZ; //Here we store the raw data read

float Gyro\_angle\_x, Gyro\_angle\_y; //Here we store the angle value obtained with Gyro data

float Gyro\_raw\_error\_x, Gyro\_raw\_error\_y; //Here we store the initial gyro data error

//Acc Variables

int acc\_error=0; //We use this variable to only calculate once the Acc data error

float rad\_to\_deg = 180/3.141592654; //This value is for pasing from radians to degrees values

float Acc\_rawX, Acc\_rawY, Acc\_rawZ; //Here we store the raw data read

float Acc\_angle\_x, Acc\_angle\_y; //Here we store the angle value obtained with Acc data

float Acc\_angle\_error\_x, Acc\_angle\_error\_y; //Here we store the initial Acc data error

float Total\_angle\_x, Total\_angle\_y;

//More variables for the code

int i;

int mot\_activated=0;

long activate\_count=0;

long des\_activate\_count=0;

//////////////////////////////PID FOR ROLL///////////////////////////

float roll\_PID, pwm\_L\_F, pwm\_L\_B, pwm\_R\_F, pwm\_R\_B, roll\_error, roll\_previous\_error;

float roll\_pid\_p=0;

float roll\_pid\_i=0;

float roll\_pid\_d=0;

///////////////////////////////ROLL PID CONSTANTS////////////////////

double roll\_kp=0.7;//3.55

double roll\_ki=0.006;//0.003

double roll\_kd=1.2;//2.05

float roll\_desired\_angle = 0; //This is the angle in which we whant the

//////////////////////////////PID FOR PITCH//////////////////////////

float pitch\_PID, pitch\_error, pitch\_previous\_error;

float pitch\_pid\_p=0;

float pitch\_pid\_i=0;

float pitch\_pid\_d=0;

///////////////////////////////PITCH PID CONSTANTS///////////////////

double pitch\_kp=0.72;//3.55

double pitch\_ki=0.006;//0.003

double pitch\_kd=1.22;//2.05

float pitch\_desired\_angle = 0; //This is the angle in which we whant the

void setup() {

PCICR |= (1 \&lt;\&lt; PCIE0); //enable PCMSK0 scan

PCMSK0 |= (1 \&lt;\&lt; PCINT0); //Set pin D8 trigger an interrupt on state change.

PCMSK0 |= (1 \&lt;\&lt; PCINT1); //Set pin D9 trigger an interrupt on state change.

PCMSK0 |= (1 \&lt;\&lt; PCINT2); //Set pin D10 trigger an interrupt on state change.

PCMSK0 |= (1 \&lt;\&lt; PCINT4); //Set pin D12 trigger an interrupt on state change.

DDRB |= B00100000; //D13 as output

PORTB &amp;= B11011111; //D13 set to LOW

L\_F\_prop.attach(4); //left front motor

L\_B\_prop.attach(5); //left back motor

R\_F\_prop.attach(7); //right front motor

R\_B\_prop.attach(6); //right back motor

/\*in order to make sure that the ESCs won&#39;t enter into config mode

\*I send a 1000us pulse to each ESC.\*/

L\_F\_prop.writeMicroseconds(1000);

L\_B\_prop.writeMicroseconds(1000);

R\_F\_prop.writeMicroseconds(1000);

R\_B\_prop.writeMicroseconds(1000);

Wire.begin(); //begin the wire comunication

Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)

Wire.write(0x6B); //make the reset (place a 0 into the 6B register)

Wire.write(0x00);

Wire.endTransmission(true); //end the transmission

Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)

Wire.write(0x1B); //We want to write to the GYRO\_CONFIG register (1B hex)

Wire.write(0x10); //Set the register bits as 00010000 (100dps full scale)

Wire.endTransmission(true); //End the transmission with the gyro

Wire.beginTransmission(0x68); //Start communication with the address found during search.

Wire.write(0x1C); //We want to write to the ACCEL\_CONFIG register (1A hex)

Wire.write(0x10); //Set the register bits as 00010000 (+/- 8g full scale range)

Wire.endTransmission(true);

Serial.begin(9600);

delay(1000);

time = millis(); //Start counting time in milliseconds

/\*Here we calculate the gyro data error before we start the loop

\* I make the mean of 200 values, that should be enough\*/

if(gyro\_error==0)

{

for(int i=0; i\&lt;200; i++)

{

Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)

Wire.write(0x43); //First adress of the Gyro data

Wire.endTransmission(false);

Wire.requestFrom(0x68,4,true); //We ask for just 4 registers

Gyr\_rawX=Wire.read()\&lt;\&lt;8|Wire.read(); //Once again we shif and sum

Gyr\_rawY=Wire.read()\&lt;\&lt;8|Wire.read();

/\*---X---\*/

Gyro\_raw\_error\_x = Gyro\_raw\_error\_x + (Gyr\_rawX/32.8);

/\*---Y---\*/

Gyro\_raw\_error\_y = Gyro\_raw\_error\_y + (Gyr\_rawY/32.8);

if(i==199)

{

Gyro\_raw\_error\_x = Gyro\_raw\_error\_x/200;

Gyro\_raw\_error\_y = Gyro\_raw\_error\_y/200;

gyro\_error=1;

}

}

}//end of gyro error calculation

/\*Here we calculate the acc data error before we start the loop

\* I make the mean of 200 values, that should be enough\*/

if(acc\_error==0)

{

for(int a=0; a\&lt;200; a++)

{

Wire.beginTransmission(0x68);

Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX

Wire.endTransmission(false);

Wire.requestFrom(0x68,6,true);

Acc\_rawX=(Wire.read()\&lt;\&lt;8|Wire.read())/4096.0 ; //each value needs two registres

Acc\_rawY=(Wire.read()\&lt;\&lt;8|Wire.read())/4096.0 ;

Acc\_rawZ=(Wire.read()\&lt;\&lt;8|Wire.read())/4096.0 ;

/\*---X---\*/

Acc\_angle\_error\_x = Acc\_angle\_error\_x + ((atan((Acc\_rawY)/sqrt(pow((Acc\_rawX),2) + pow((Acc\_rawZ),2)))\*rad\_to\_deg));

/\*---Y---\*/

Acc\_angle\_error\_y = Acc\_angle\_error\_y + ((atan(-1\*(Acc\_rawX)/sqrt(pow((Acc\_rawY),2) + pow((Acc\_rawZ),2)))\*rad\_to\_deg));

if(a==199)

{

Acc\_angle\_error\_x = Acc\_angle\_error\_x/200;

Acc\_angle\_error\_y = Acc\_angle\_error\_y/200;

acc\_error=1;

}

}

}//end of acc error calculation

}//end of setup loop

void loop() {

/////////////////////////////I M U/////////////////////////////////////

timePrev = time; // the previous time is stored before the actual time read

time = millis(); // actual time read

elapsedTime = (time - timePrev) / 1000;

/\*The tiemStep is the time that elapsed since the previous loop.

\*This is the value that we will use in the formulas as &quot;elapsedTime&quot;

\*in seconds. We work in ms so we have to divide the value by 1000

to obtain seconds\*/

/\*Reed the values that the accelerometre gives.

\* We know that the slave adress for this IMU is 0x68 in

\* hexadecimal. For that in the RequestFrom and the

\* begin functions we have to put this value.\*/

//////////////////////////////////////Gyro read/////////////////////////////////////

Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)

Wire.write(0x43); //First adress of the Gyro data

Wire.endTransmission(false);

Wire.requestFrom(0x68,4,true); //We ask for just 4 registers

Gyr\_rawX=Wire.read()\&lt;\&lt;8|Wire.read(); //Once again we shif and sum

Gyr\_rawY=Wire.read()\&lt;\&lt;8|Wire.read();

/\*Now in order to obtain the gyro data in degrees/seconds we have to divide first

the raw value by 32.8 because that&#39;s the value that the datasheet gives us for a 1000dps range\*/

/\*---X---\*/

Gyr\_rawX = (Gyr\_rawX/32.8) - Gyro\_raw\_error\_x;

/\*---Y---\*/

Gyr\_rawY = (Gyr\_rawY/32.8) - Gyro\_raw\_error\_y;

/\*Now we integrate the raw value in degrees per seconds in order to obtain the angle

\* If you multiply degrees/seconds by seconds you obtain degrees \*/

/\*---X---\*/

Gyro\_angle\_x = Gyr\_rawX\*elapsedTime;

/\*---X---\*/

Gyro\_angle\_y = Gyr\_rawY\*elapsedTime;

//////////////////////////////////////Acc read/////////////////////////////////////

Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)

Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX

Wire.endTransmission(false); //keep the transmission and next

Wire.requestFrom(0x68,6,true); //We ask for next 6 registers starting withj the 3B

/\*We have asked for the 0x3B register. The IMU will send a brust of register.

\* The amount of register to read is specify in the requestFrom function.

\* In this case we request 6 registers. Each value of acceleration is made out of

\* two 8bits registers, low values and high values. For that we request the 6 of them

\* and just make then sum of each pair. For that we shift to the left the high values

\* register (\&lt;\&lt;) and make an or (|) operation to add the low values.

If we read the datasheet, for a range of+-8g, we have to divide the raw values by 4096\*/

Acc\_rawX=(Wire.read()\&lt;\&lt;8|Wire.read())/4096.0 ; //each value needs two registres

Acc\_rawY=(Wire.read()\&lt;\&lt;8|Wire.read())/4096.0 ;

Acc\_rawZ=(Wire.read()\&lt;\&lt;8|Wire.read())/4096.0 ;

/\*Now in order to obtain the Acc angles we use euler formula with acceleration values

after that we substract the error value found before\*/

/\*---X---\*/

Acc\_angle\_x = (atan((Acc\_rawY)/sqrt(pow((Acc\_rawX),2) + pow((Acc\_rawZ),2)))\*rad\_to\_deg) - Acc\_angle\_error\_x;

/\*---Y---\*/

Acc\_angle\_y = (atan(-1\*(Acc\_rawX)/sqrt(pow((Acc\_rawY),2) + pow((Acc\_rawZ),2)))\*rad\_to\_deg) - Acc\_angle\_error\_y;

//////////////////////////////////////Total angle and filter/////////////////////////////////////

/\*---X axis angle---\*/

Total\_angle\_x = 0.98 \*(Total\_angle\_x + Gyro\_angle\_x) + 0.02\*Acc\_angle\_x;

/\*---Y axis angle---\*/

Total\_angle\_y = 0.98 \*(Total\_angle\_y + Gyro\_angle\_y) + 0.02\*Acc\_angle\_y;

/\*///////////////////////////P I D///////////////////////////////////\*/

roll\_desired\_angle = map(input\_ROLL,1000,2000,-10,10);

pitch\_desired\_angle = map(input\_PITCH,1000,2000,-10,10);

/\*First calculate the error between the desired angle and

\*the real measured angle\*/

roll\_error = Total\_angle\_y - roll\_desired\_angle;

pitch\_error = Total\_angle\_x - pitch\_desired\_angle;

/\*Next the proportional value of the PID is just a proportional constant

\*multiplied by the error\*/

roll\_pid\_p = roll\_kp\*roll\_error;

pitch\_pid\_p = pitch\_kp\*pitch\_error;

/\*The integral part should only act if we are close to the

desired position but we want to fine tune the error. That&#39;s

why I&#39;ve made a if operation for an error between -2 and 2 degree.

To integrate we just sum the previous integral value with the

error multiplied by the integral constant. This will integrate (increase)

the value each loop till we reach the 0 point\*/

if(-3 \&lt; roll\_error \&lt;3)

{

roll\_pid\_i = roll\_pid\_i+(roll\_ki\*roll\_error);

}

if(-3 \&lt; pitch\_error \&lt;3)

{

pitch\_pid\_i = pitch\_pid\_i+(pitch\_ki\*pitch\_error);

}

/\*The last part is the derivate. The derivate acts upon the speed of the error.

As we know the speed is the amount of error that produced in a certain amount of

time divided by that time. For taht we will use a variable called previous\_error.

We substract that value from the actual error and divide all by the elapsed time.

Finnaly we multiply the result by the derivate constant\*/

roll\_pid\_d = roll\_kd\*((roll\_error - roll\_previous\_error)/elapsedTime);

pitch\_pid\_d = pitch\_kd\*((pitch\_error - pitch\_previous\_error)/elapsedTime);

/\*The final PID values is the sum of each of this 3 parts\*/

roll\_PID = roll\_pid\_p + roll\_pid\_i + roll\_pid\_d;

pitch\_PID = pitch\_pid\_p + pitch\_pid\_i + pitch\_pid\_d;

/\*We know taht the min value of PWM signal is 1000us and the max is 2000. So that

tells us that the PID value can/s oscilate more than -1000 and 1000 because when we

have a value of 2000us the maximum value taht we could substract is 1000 and when

we have a value of 1000us for the PWM signal, the maximum value that we could add is 1000

to reach the maximum 2000us. But we don&#39;t want to act over the entire range so -+400 should be enough\*/

if(roll\_PID \&lt; -400){roll\_PID=-400;}

if(roll\_PID \&gt; 400) {roll\_PID=400; }

if(pitch\_PID \&lt; -4000){pitch\_PID=-400;}

if(pitch\_PID \&gt; 400) {pitch\_PID=400;}

/\*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value\*/

pwm\_R\_F = 115 + input\_THROTTLE - roll\_PID - pitch\_PID;

pwm\_R\_B = 115 + input\_THROTTLE - roll\_PID + pitch\_PID;

pwm\_L\_B = 115 + input\_THROTTLE + roll\_PID + pitch\_PID;

pwm\_L\_F = 115 + input\_THROTTLE + roll\_PID - pitch\_PID;

/\*Once again we map the PWM values to be sure that we won&#39;t pass the min

and max values. Yes, we&#39;ve already maped the PID values. But for example, for

throttle value of 1300, if we sum the max PID value we would have 2300us and

that will mess up the ESC.\*/

//Right front

if(pwm\_R\_F \&lt; 1100)

{

pwm\_R\_F= 1100;

}

if(pwm\_R\_F \&gt; 2000)

{

pwm\_R\_F=2000;

}

//Left front

if(pwm\_L\_F \&lt; 1100)

{

pwm\_L\_F= 1100;

}

if(pwm\_L\_F \&gt; 2000)

{

pwm\_L\_F=2000;

}

//Right back

if(pwm\_R\_B \&lt; 1100)

{

pwm\_R\_B= 1100;

}

if(pwm\_R\_B \&gt; 2000)

{

pwm\_R\_B=2000;

}

//Left back

if(pwm\_L\_B \&lt; 1100)

{

pwm\_L\_B= 1100;

}

if(pwm\_L\_B \&gt; 2000)

{

pwm\_L\_B=2000;

}

roll\_previous\_error = roll\_error; //Remember to store the previous error.

pitch\_previous\_error = pitch\_error; //Remember to store the previous error.

/\*

Serial.print(&quot;RF: &quot;);

Serial.print(pwm\_R\_F);

Serial.print(&quot; | &quot;);

Serial.print(&quot;RB: &quot;);

Serial.print(pwm\_R\_B);

Serial.print(&quot; | &quot;);

Serial.print(&quot;LB: &quot;);

Serial.print(pwm\_L\_B);

Serial.print(&quot; | &quot;);

Serial.print(&quot;LF: &quot;);

Serial.print(pwm\_L\_F);

Serial.print(&quot; | &quot;);

Serial.print(&quot;Xº: &quot;);

Serial.print(Total\_angle\_x);

Serial.print(&quot; | &quot;);

Serial.print(&quot;Yº: &quot;);

Serial.print(Total\_angle\_y);

Serial.println(&quot; &quot;);

\*/

/\*now we can write the values PWM to the ESCs only if the motor is activated

\*/

if(mot\_activated)

{

L\_F\_prop.writeMicroseconds(pwm\_L\_F);

L\_B\_prop.writeMicroseconds(pwm\_L\_B);

R\_F\_prop.writeMicroseconds(pwm\_R\_F);

R\_B\_prop.writeMicroseconds(pwm\_R\_B);

}

if(!mot\_activated)

{

L\_F\_prop.writeMicroseconds(1000);

L\_B\_prop.writeMicroseconds(1000);

R\_F\_prop.writeMicroseconds(1000);

R\_B\_prop.writeMicroseconds(1000);

}

if(input\_THROTTLE \&lt; 1100 &amp;&amp; input\_YAW \&gt; 1800 &amp;&amp; !mot\_activated)

{

if(activate\_count==200)

{

mot\_activated=1;

PORTB |= B00100000; //D13 LOW

}

activate\_count=activate\_count+1;

}

if(!(input\_THROTTLE \&lt; 1100 &amp;&amp; input\_YAW \&gt; 1800) &amp;&amp; !mot\_activated)

{

activate\_count=0;

}

if(input\_THROTTLE \&lt; 1100 &amp;&amp; input\_YAW \&lt; 1100 &amp;&amp; mot\_activated)

{

if(des\_activate\_count==300)

{

mot\_activated=0;

PORTB &amp;= B11011111; //D13 LOW

}

des\_activate\_count=des\_activate\_count+1;

}

if(!(input\_THROTTLE \&lt; 1100 &amp;&amp; input\_YAW \&lt; 1100) &amp;&amp; mot\_activated)

{

des\_activate\_count=0;

}

}

ISR(PCINT0\_vect){

//First we take the current count value in micro seconds using the micros() function

current\_count = micros();

///////////////////////////////////////Channel 1

if(PINB &amp; B00000001){ //We make an AND with the pin state register, We verify if pin 8 is HIGH???

if(last\_CH1\_state == 0){ //If the last state was 0, then we have a state change...

last\_CH1\_state = 1; //Store the current state into the last state for the next loop

counter\_1 = current\_count; //Set counter\_1 to current value.

}

}

else if(last\_CH1\_state == 1){ //If pin 8 is LOW and the last state was HIGH then we have a state change

last\_CH1\_state = 0; //Store the current state into the last state for the next loop

input\_ROLL = current\_count - counter\_1; //We make the time difference. Channel 1 is current\_time - timer\_1.

}

///////////////////////////////////////Channel 2

if(PINB &amp; B00000010 ){ //pin D9 -- B00000010

if(last\_CH2\_state == 0){

last\_CH2\_state = 1;

counter\_2 = current\_count;

}

}

else if(last\_CH2\_state == 1){

last\_CH2\_state = 0;

input\_PITCH = current\_count - counter\_2;

}

///////////////////////////////////////Channel 3

if(PINB &amp; B00000100 ){ //pin D10 - B00000100

if(last\_CH3\_state == 0){

last\_CH3\_state = 1;

counter\_3 = current\_count;

}

}

else if(last\_CH3\_state == 1){

last\_CH3\_state = 0;

input\_THROTTLE = current\_count - counter\_3;

}

///////////////////////////////////////Channel 4

if(PINB &amp; B00010000 ){ //pin D12 -- B00010000

if(last\_CH4\_state == 0){

last\_CH4\_state = 1;

counter\_4 = current\_count;

}

}

else if(last\_CH4\_state == 1){

last\_CH4\_state = 0;

input\_YAW = current\_count - counter\_4;

}

}

## **Radio Transmitter Arduino Code**

/\*A basic 4 channel transmitter using the nRF24L01 module.

\*

\*/

#include \&lt;SPI.h\&gt;

#include \&lt;nRF24L01.h\&gt;

#include \&lt;RF24.h\&gt;

/\*Create a unique pipe out. The receiver has to wear the same unique code\*/

const uint64\_t pipeOut = 0xE8E8F0F0E1LL; //IMPORTANT: The same as in the receiver

RF24 radio(9, 10); // select CSN pin

// The sizeof this struct should not exceed 32 bytes

// This gives us up to 32 8 bits channals

struct MyData {

byte throttle;

byte yaw;

byte pitch;

byte roll;

byte AUX1;

byte AUX2;

};

MyData data;

void resetData()

{

//This are the start values of each channal

// Throttle is 0 in order to stop the motors

//127 is the middle value of the 10ADC.

data.throttle = 0;

data.yaw = 127;

data.pitch = 127;

data.roll = 127;

data.AUX1 = 0;

data.AUX2 = 0;

}

void setup()

{

//Start everything up

radio.begin();

radio.setAutoAck(false);

radio.setDataRate(RF24\_250KBPS);

radio.openWritingPipe(pipeOut);

resetData();

}

/\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*/

// Returns a corrected value for a joystick position that takes into account

// the values of the outer extents and the middle of the joystick range.

int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)

{

val = constrain(val, lower, upper);

if ( val \&lt; middle )

val = map(val, lower, middle, 0, 128);

else

val = map(val, middle, upper, 128, 255);

return ( reverse ? 255 - val : val );

}

void loop()

{

// The calibration numbers used here should be measured

// for your joysticks till they send the correct values.

data.throttle = mapJoystickValues( analogRead(A0), 13, 524, 1015, true );

data.yaw = mapJoystickValues( analogRead(A1), 1, 505, 1020, true );

data.pitch = mapJoystickValues( analogRead(A2), 12, 544, 1021, true );

data.roll = mapJoystickValues( analogRead(A3), 34, 522, 1020, true );

data.AUX1 = digitalRead(4); //The 2 toggle switches

data.AUX2 = digitalRead(5);

radio.write(&amp;data, sizeof(MyData));

}

## **Radio Receiver Arduino Code**

/\*

\* A basic receiver test for the nRF24L01 module to receive 6 channels send a ppm sum

\* with all of them on digital pin D2.

\*/

#include \&lt;SPI.h\&gt;

#include \&lt;nRF24L01.h\&gt;

#include \&lt;RF24.h\&gt;

////////////////////// PPM CONFIGURATION//////////////////////////

#define channel\_number 6 //set the number of channels

#define sigPin 2 //set PPM signal output pin on the arduino

#define PPM\_FrLen 27000 //set the PPM frame length in microseconds (1ms = 1000µs)

#define PPM\_PulseLen 400 //set the pulse length

//////////////////////////////////////////////////////////////////

int ppm[channel\_number];

const uint64\_t pipeIn = 0xE8E8F0F0E1LL;

RF24 radio(9, 10);

// The sizeof this struct should not exceed 32 bytes

struct MyData {

byte throttle;

byte yaw;

byte pitch;

byte roll;

byte AUX1;

byte AUX2;

};

MyData data;

void resetData()

{

// &#39;safe&#39; values to use when no radio input is detected

data.throttle = 0;

data.yaw = 127;

data.pitch = 127;

data.roll = 127;

data.AUX1 = 0;

data.AUX2= 0;

setPPMValuesFromData();

}

void setPPMValuesFromData()

{

ppm[0] = map(data.throttle, 0, 255, 1000, 2000);

ppm[1] = map(data.yaw, 0, 255, 1000, 2000);

ppm[2] = map(data.pitch, 0, 255, 1000, 2000);

ppm[3] = map(data.roll, 0, 255, 1000, 2000);

ppm[4] = map(data.AUX1, 0, 1, 1000, 2000);

ppm[5] = map(data.AUX2, 0, 1, 1000, 2000);

}

/\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*/

void setupPPM() {

pinMode(sigPin, OUTPUT);

digitalWrite(sigPin, 0); //set the PPM signal pin to the default state (off)

cli();

TCCR1A = 0; // set entire TCCR1 register to 0

TCCR1B = 0;

OCR1A = 100; // compare match register (not very important, sets the timeout for the first interrupt)

TCCR1B |= (1 \&lt;\&lt; WGM12); // turn on CTC mode

TCCR1B |= (1 \&lt;\&lt; CS11); // 8 prescaler: 0,5 microseconds at 16mhz

TIMSK1 |= (1 \&lt;\&lt; OCIE1A); // enable timer compare interrupt

sei();

}

void setup()

{

resetData();

setupPPM();

// Set up radio module

radio.begin();

radio.setDataRate(RF24\_250KBPS); // Both endpoints must have this set the same

radio.setAutoAck(false);

radio.openReadingPipe(1,pipeIn);

radio.startListening();

}

/\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*/

unsigned long lastRecvTime = 0;

void recvData()

{

while ( radio.available() ) {

radio.read(&amp;data, sizeof(MyData));

lastRecvTime = millis();

}

}

/\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*/

void loop()

{

recvData();

unsigned long now = millis();

if ( now - lastRecvTime \&gt; 1000 ) {

// signal lost?

resetData();

}

setPPMValuesFromData();

}

/\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*/

#define clockMultiplier 2 // set this to 2 if you are using a 16MHz arduino, leave as 1 for an 8MHz arduino

ISR(TIMER1\_COMPA\_vect){

static boolean state = true;

TCNT1 = 0;

if ( state ) {

//end pulse

PORTD = PORTD &amp; ~B00000100; // turn pin 2 off. Could also use: digitalWrite(sigPin,0)

OCR1A = PPM\_PulseLen \* clockMultiplier;

state = false;

}

else {

//start pulse

static byte cur\_chan\_numb;

static unsigned int calc\_rest;

PORTD = PORTD | B00000100; // turn pin 2 on. Could also use: digitalWrite(sigPin,1)

state = true;

if(cur\_chan\_numb \&gt;= channel\_number) {

cur\_chan\_numb = 0;

calc\_rest += PPM\_PulseLen;

OCR1A = (PPM\_FrLen - calc\_rest) \* clockMultiplier;

calc\_rest = 0;

}

else {

OCR1A = (ppm[cur\_chan\_numb] - PPM\_PulseLen) \* clockMultiplier;

calc\_rest += ppm[cur\_chan\_numb];

cur\_chan\_numb++;

}

}

}

## **ESC Calibration Arduino Code**

/\*ESC calibration sketch; author: EDWARD OSARETIN OBOH \*/

#include \&lt;Servo.h\&gt;

#define MAX\_SIGNAL 2000

#define MIN\_SIGNAL 1000

#define MOTOR\_PIN 9

int DELAY = 1000;

Servo motor;

void setup() {

Serial.begin(9600);

Serial.println(&quot;Don&#39;t forget to subscribe!&quot;);

Serial.println(&quot;ESC calibration...&quot;);

Serial.println(&quot; &quot;);

delay(1500);

Serial.println(&quot;Program begin...&quot;);

delay(1000);

Serial.println(&quot;This program will start the ESC.&quot;);

motor.attach(MOTOR\_PIN);

Serial.print(&quot;Now writing maximum output: (&quot;);Serial.print(MAX\_SIGNAL);Serial.print(&quot; us in this case)&quot;);Serial.print(&quot;\n&quot;);

Serial.println(&quot;Turn on power source, then wait 2 seconds and press any key.&quot;);

motor.writeMicroseconds(MAX\_SIGNAL);

// Wait for input

while (!Serial.available());

Serial.read();

// Send min output

Serial.println(&quot;\n&quot;);

Serial.println(&quot;\n&quot;);

Serial.print(&quot;Sending minimum output: (&quot;);Serial.print(MIN\_SIGNAL);Serial.print(&quot; us in this case)&quot;);Serial.print(&quot;\n&quot;);

motor.writeMicroseconds(MIN\_SIGNAL);

Serial.println(&quot;The ESC is calibrated&quot;);

Serial.println(&quot;----&quot;);

Serial.println(&quot;Now, type a values between 1000 and 2000 and press enter&quot;);

Serial.println(&quot;and the motor will start rotating.&quot;);

Serial.println(&quot;Send 1000 to stop the motor and 2000 for full throttle&quot;);

}

void loop() {

if (Serial.available() \&gt; 0)

{

int DELAY = Serial.parseInt();

if (DELAY \&gt; 999)

{

motor.writeMicroseconds(DELAY);

float SPEED = (DELAY-1000)/10;

Serial.print(&quot;\n&quot;);

Serial.println(&quot;Motor speed:&quot;); Serial.print(&quot; &quot;); Serial.print(SPEED); Serial.print(&quot;%&quot;);

}

}

}
