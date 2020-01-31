# OpenShoe in C for realtime use

Original Matlab algorithm from http://www.openshoe.org completely rewritten in C by me to be used in real-time application.

I load the dataset and compute it line by line to simulate incoming data in real-time. 
Easily adaptable to be integrated in an embedded system. Tested in an ESP8266 coupled with an MPU-9250.

The smoothing algorithm was not integrated because it was too much computationally expensive for the project I was working on.
Please see my other repo "OpenShoe_C" to find the smoothing algorithm rewritten in C.

I did my best to keep the same file and project structure.

The results are the same between the original algorithm and this one. See the results in the "results" folder.
I used the original data_set_1/data_inert_left.txt for testing. 

To plot result.txt use `python3 plotGraph.py`
