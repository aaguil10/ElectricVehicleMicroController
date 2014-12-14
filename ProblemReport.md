# Dream Team Acceptance Testing Problem Report

Due to unexpected delays involving the implementation of our code and access to
the Dagne vehicle for testing we are unable to perform a test on the vehicle
itself and automated unit tests have not been written. However, we will be
performing a test of the code we have written for the steering control system.

Instead of testing on the video, a test harness has been constructed using
another arduino. This means our test will have two arduinos, one is the
microcontroller as would be put in the car, the other simulates the inputs and
outputs of the vehicle. We are able to simulate the steering control and receive
the proper outputs from our microcontroller code. The outputs are confirmed by
LEDs that help visualize the PWM output for the steering system and can be
verified through serial logging from the Arduinos.

In addition, a large portion of our project is the design documentation which
can be found in the GitHub repository in the design_docs folder
(https://github.com/aaguil10/ElectricVehicleMicroController). We consider this
document as our user documentation as the user of our product is future vehicle
developers.


