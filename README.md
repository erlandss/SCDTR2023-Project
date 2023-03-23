# SCDTR2023-Project

Milestones (mark when done):

Develop network “wake-up” procedures so that all nodes are aware of each-other and
boot properly. DONE

Calibrate of the gains of distributed system model. Write a program to be deployed in
all controller nodes to make them synchronously turn on and off the luminaires. This
code then should be used to calibrate the self- and cross-coupling gains between the
several nodes in the distributed system. This procedure should be done every time the
system wakes up to prevent errors in calibration due to movement of the elements inside
the box.

Implement the Hub function to serve the listed commands. Write and test a
microcontroller program that routes messages between the PC and other controller
nodes. Interface this program with the Serial Monitor.

Implement the distributed controller given in the theory lectures using C++ classes.
Define appropriate communication protocol to exchange the required messages between
the nodes.

Show how the controllers react to changes in desk occupation and external disturbances.
Consider two cases: identical or different costs in the energy minimization criteria.
Compare this with the non-coordinated controller when no communications are
available.

Write a asynchronous socket based server using Boost::ASIO C++ library. Suitable
TCP/IP and UDP demo clients will be provided.

Implement in the server a console mode that accepts user commands and displays
information in the format specified previously.

Implement TCP/IP and/or UDP services to allow the clients to perform the safe
operations remotely as in the console (local) model.
