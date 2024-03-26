# Underactuated Tiltrotor Aircraft Project

## Overview

This repository contains the code and documentation for the capstone design project of seven aerospace engineering students from Concordia University. The project aimed to create an underactuated tiltrotor aircraft inspired by the [NEMO tiltrotor UAV](https://www.youtube.com/watch?v=_jHHTOk8GEQ&ab_channel=STEEVIE2376). The aircraft is designed to achieve vertical takeoff and landing, horizontal cruising, and hovering capabilities through a propulsion system that eliminates the need for servo actuators and traditional tilt mechanisms.

Here is the video of our tiltrotor test stand Dory to prove that servo motors are not needed to vector the thrust of the propellers!

<p align="center">
    <a href="https://github.com/Sanassah/Underactuated-Tiltrotor/assets/89364990/14d8e611-1522-433e-9fc2-57b47e61e484">
        <img src="https://github.com/Sanassah/Underactuated-Tiltrotor/assets/89364990/14d8e611-1522-433e-9fc2-57b47e61e484" alt="Tilt Rotor Test Video" width="560" height="315">
    </a>
</p>


## Key Features

- **Unique Engine Design:** The aircraft features distinctive motors and propellers from [Vertiq](https://www.vertiq.co/) that replaces the traditional tilt mechanisms of servos with a pulsing voltage and hinge capable of angling the propellers with each acceleration and deceleration in each revolution. 

<p align="center">
    <img src="https://github.com/Sanassah/Underactuated-Tiltrotor/assets/89364990/f7f2a61d-906c-47e4-a8c5-7df77d079b6d" width="350" />
</p>


- **Custom Flight Controller:** Due to the unique configuration and technology of the aircraft, for which no pre-made flight computer is suitable, we opted to develop a custom flight controller using a Teensy 4.1 microcontroller. [DRehmFlight open source flight controller code](https://github.com/nickrehm/dRehmFlight) by Nick Rehm served as a base for our attitude control design.


## Progress so far!

Many many hours of design and assembly! (2024-02-02):

<img src="https://github.com/Sanassah/Underactuated-Tiltrotor/assets/89364990/207bde94-2ea4-4e9d-ad4d-8cb5cf5c115a" width="400" />

Assembly of the 3D printed parts on the attitude test stand Marlin (2024-02-04):

<img src="https://github.com/Sanassah/Underactuated-Tiltrotor/assets/89364990/0c572364-1377-48d9-8e12-6d73bd850a50" width="300" />

Assembly of the electrical system (2024-02-09):

<img src="https://github.com/Sanassah/Underactuated-Tiltrotor/assets/89364990/415006fa-6b79-466a-9398-c79e68e9142e" width="500" />

Assembled Drone (2024-02-11):

<img src="https://github.com/Sanassah/Underactuated-Tiltrotor/assets/89364990/b53f768a-0b1b-4778-8718-e72504cf0644" width="500" />

Attitude test attached to ground (2024-03-11):

<img src="https://github.com/Sanassah/Underactuated-Tiltrotor/assets/89364990/23f82de0-f94e-4159-a194-c9a30ec939ff" width="400" />

Bunny Hop! Unfortunetaly by the end of testing day, we chipped the props and it seemes like we were missing some thrust (2024-03-11):

<img src="https://github.com/Sanassah/Underactuated-Tiltrotor/assets/89364990/c0950ae3-f504-434b-a5d8-8251271f84d8" width="500" />

First Hover test!!! (2024-03-25):

<img src="https://github.com/Sanassah/Underactuated-Tiltrotor/assets/89364990/50035ca9-4d8d-413a-8b20-f5b3887c2c20" width="500" />

Tried to gain some horizontal speed and fatal crash unfortunately because nacelles touched the ground and thrust to weight ratio was too low to recover the aircraft, props got damaged and they are pretty expensive so testing will be paused for now (2024-03-25):

<img src="https://github.com/Sanassah/Underactuated-Tiltrotor/assets/89364990/1ce4f9af-1b4b-4f86-b538-755c065a02ae" width="500" />

