# Python implementation of SMTO

This folder contains the python implementation of SMTO, which was used for the experiments with manipulators in the [paper](https://arxiv.org/abs/2003.07054).
Since the optimization with SMTO is a stochastic process, the result will be different for each run.

## Requirements

numpy == 1.14.5 \
scipy == 1.4.1 \
sklearn == 0.22.1

CoppeliaSim (formerly V-REP) [[download](https://www.coppeliarobotics.com/downloads)]

## Descripyion

`SMTO_kuka_bag.py` shows the behavior of SMTO in the case where the start and goal configurations are given and fixed.

`SMTO_kuka_bottle.py` shows the behavior of SMTO in the case where the start congifuration is given and fixed and the goal configuration has a rotaional freedom.

## How to use

For the task with a bag

1. Open CoppeliaSim and load `KUKA_jaco_bag.ttt`

2. run `python SMTO_kuka_bag.py`

For the task with a bottle

1. Open CoppeliaSim and load `KUKA_jaco_bottle.ttt`

2. run `python SMTOend_kuka_bottle.py`

All the codes in this repository are shared under  a CC Attribution-NonCommerical license.  
