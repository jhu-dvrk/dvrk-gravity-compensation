## Overview
This _dvrk\_gravity\_compensation_ package is designed for gravity compensation(GC) of Master Tool Manipulator(MTM) for dVRK. Compared to former work, our repository solves following problems:

* Elastic force and friction modeling in dynamics model 
* New data collection strategy of 7 DOF serial manipulator for further estimation 
* Multi-steps least square estimation
* Low-Friction Zero-Gravity controller

## Running Program
-----
### 1. launch dVRK console

Open a terminal to start roscore

```
$ roscore
```

Open another terminal to launch dVRK console
```
$ qlacloserelays
#use a console config that contains the config file names of MTML and MTMR
$rosrun dvrk_robot dvrk_console_json -j <path_to_your_console_config.json>
```
After opening console, press `home` button to turn on MTM arms and move them to home position.

-----
### 2.Initialze MATLAB

Open MATLAB and go to the folder, _dvrk\_gravity\_compensation_. 

Then initialize the system:
```
addpath('<path-to-dvrk_matlab>');
rosinit;
```

-----
### 3. Runing MATLAB Script Program


Run the whole program in one command
```
mtm_gc_controller = gc_program('<ARM-NAME>','<Serial-Number>')

```
For example:
```
% For MTML
mtml_gc_controller = gc_program('MTML','12345')
% For MTMR
mtmr_gc_controller = gc_program('MTMR','54321')
```

Then the program starts. It will go through process: 

**A) [wizard\_config\_dataCollection]**,

**B) [dataCollection]**, 

**C) [mlse]** 

**D) [gc_controller]**.

-----
### A) **[wizard\_config\_dataCollection]** (Manually, Need user input)
In this process, user need to set the customized joint limits for further dataCollection process. Going through this process is extremely necessary because we don't want our MTMs hit the environments. For the reason that each MTM can be installed in different environment, we use this program to set some joints limits.

After you step into this process, you will go through some instructions to set joint limits and then execute the collision checking. 

-----

#### A) Step #1:
MATLAB console:
<p align="center">
 <img src="https://raw.githubusercontent.com/wiki/CUHK-BRME/dvrk_Gravity_Compensation/images/wizard_1.png?token=ASrIorh7-N4opBUSKuab-Wz4mCehXyQQks5cDhdqwA%3D%3D" width="1000"  />
</p>

The console will show some information of instructions， including instruction goal, MTM arm, Joint No which will be moved, the customized and current value and keyboard instruction. 

- To increase joint angle by 1 degree, type _i_ and return. 

- To decrease joint angle by 1 degree, type _d_ and type return.

- To set joint limit value, type _f_ and type return

Set the joint limit when distal link of MTM is around 10cm away from top panel.
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/CUHK-BRME/dvrk_Gravity_Compensation/images/wizard_1_real.png?token=ASrIotpndAu-qZ6UyQ_vogdFo3HS2ruTks5cDhd9wA%3D%3D" width="500"  />
 </p>
 

----------
#### A) Step #2:
MATLAB console:
<p align="center">
 <img src="https://raw.githubusercontent.com/wiki/CUHK-BRME/dvrk_Gravity_Compensation/images/wizard_2.png?token=ASrIokYcS1bZtwM5CyAi7RHt7Jswr6jGks5cDheUwA%3D%3D" width="1000"  />
 </p>

Set the joint limit when distal link of MTM is around 10cm away from front panel.
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/CUHK-BRME/dvrk_Gravity_Compensation/images/wizard_2_real.png?token=ASrIopFx8cMTVktaWsePYzzgzlNoUMU1ks5cDhmUwA%3D%3D" width="500"  />
 </p>


----------
#### A) Step #3:
MATLAB console:
<p align="center">
  <img src="https://raw.githubusercontent.com/wiki/CUHK-BRME/dvrk_Gravity_Compensation/images/wizard_3.png?token=ASrIoktfBCnfEPH640inE1-jdt1WsGeSks5cDhl-wA%3D%3D" width="1000"  />
 </p>

Set the joint limit when distal link of MTM is around 10cm away from top panel.
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/CUHK-BRME/dvrk_Gravity_Compensation/images/wizard_3_real.png?token=ASrIonVuXOktKhX_ZgD-kqdp2Sp56rxJks5cDhmTwA%3D%3D" width="500"  />
 </p>


----------
#### A) Step #4:
MATLAB console:
<p align="center">
   <img src="https://raw.githubusercontent.com/wiki/CUHK-BRME/dvrk_Gravity_Compensation/images/wizard_4.png?token=ASrIoj8QHQE_mHH0mAbmgzOIRTQ_rZ-nks5cDhnpwA%3D%3D" width="1000"  />
 </p>

Set the joint limit when distal link of MTM is around 10cm away from top panel.
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/CUHK-BRME/dvrk_Gravity_Compensation/images/wizard_3_real.png?token=ASrIonVuXOktKhX_ZgD-kqdp2Sp56rxJks5cDhmTwA%3D%3D" width="500"  />
 </p>


----------
#### A) Step #5:
MATLAB console:
<p align="center">
 <img src="https://raw.githubusercontent.com/wiki/CUHK-BRME/dvrk_Gravity_Compensation/images/wizard_5.png?token=ASrIovPD2GhvkdoHHB52zmzj0j3fXaJtks5cDhq5wA%3D%3D" width="1000"  />
 </p>
 
Set the joint limit when distal link of MTM is around 10cm away from left panel.
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/CUHK-BRME/dvrk_Gravity_Compensation/images/wizard_5_real.png?token=ASrIopk4PBKxTDoAf61E5L3QA4Fv8BoVks5cDhqBwA%3D%3D" width="500"  />
 </p>


----------
#### A) Step #6:
MATLAB console:
<p align="center">
  <img src="https://raw.githubusercontent.com/wiki/CUHK-BRME/dvrk_Gravity_Compensation/images/wizard_6.png?token=ASrIor51OQxTuR7X4bkhmTPQQAEsFq33ks5cDhrlwA%3D%3D" width="1000"  />
 </p>
 
Set the joint limit when distal link of MTM is around 10cm away from right panel.
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/CUHK-BRME/dvrk_Gravity_Compensation/images/wizard_6_real.png?token=ASrIopB1-CELTkOYA2TeFx73aCFbLjFUks5cDhr5wA%3D%3D" width="500"  />
 </p>

Afterwards, it will execute **collision checking**. If you set the limit properly according to the previous instructions, MTM will not hit the environment. But if it unluckily hits the environment by mistakes, push E-button immediately.  

**Collision checking** is making MTM to move according to preset trajectory which will be apply in dataCollection. Therefore, if collision checking passes, MTM in the dataCollection process will be free from collision.  

### B) **[dataCollection]** (Automatic):
In this process, dataCollection of MTM will be executed. This usually spend about 1hour. (No user input is required)

### C) Process#3 **[mlse]** (Automatic):
In this process, the dynamic parameters will be calculated by multi-steps least square estimation. (No user input is required)

### D) Process#4 **[gc_controller]** (Automatic):
In this process, gravity compensation controller will be applied by loading the dynamics parameters.(No user input is required)


After finishing all 4 processes, user is able to move MTM in Gravity-Compensation mode.

## Loading in dVRK console
After you finish the first 3 process. the program will generate a file 

**..\GC_Data_stable\<ARM_NAME>_<SN>\<date&time>\gc-\<ARM-Name\>-\<SN\>.json**
 
 (For example: ..\GC_Data_stable\MTML_41878\November-30-2018-10:57:53\gc-MTML-41878.json). 

You can load the output file to Cpp version of gravity compensation of dVRK. Copy the file to "<path-to-dvrk-src>\cisst-saw\sawIntuitiveResearchKit\share". In the console JSON file, you need to edit the value of field name "gravity-compensation" :

For example: 

              gravity-compensation":"gc-MTML-12345.json"
              
Here is the template for [console json file](https://github.com/CUHK-BRME/dvrk_Gravity_Compensation/blob/V1.0/console-MTMR-MTML.json)



## Contact
Feel free to contact us.  

Hongbin LIN:  [hongbinlin@cuhk.edu.hk](hongbinlin@cuhk.edu.hk)

Vincent Hui: [vincent.hui@cuhk.edu.hk](vincent.hui@cuhk.edu.hk) 

## Acknowledgements
The software has been developed with the support of BRME,CUHK.

