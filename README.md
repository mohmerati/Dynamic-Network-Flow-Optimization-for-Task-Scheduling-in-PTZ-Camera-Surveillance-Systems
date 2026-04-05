# Dynamic Network Flow Optimization for Task Scheduling in PTZ Camera Surveillance Systems

This repository contains MATLAB code for real-time scheduling and control of Pan-Tilt-Zoom (PTZ) cameras in dynamic surveillance environments. The method combines Kalman-filter-based motion prediction with a dynamic network flow optimization model to schedule camera actions over a short planning horizon. The framework also includes a group-tracking extension that clusters nearby targets so that multiple objects can be captured within a single zoomed-in interrogation.

The goal is to improve smart video surveillance in crowded and time-varying environments by increasing target coverage, reducing average wait time before interrogation, and minimizing missed objects.

## Overview

PTZ cameras are flexible sensing platforms, but controlling them efficiently in dynamic scenes is challenging. A useful controller must decide:

- which target or region each camera should observe,
- when a camera should stay in wide-area mode versus zoom in,
- how to prioritize targets that may soon leave the scene,
- and how to coordinate multiple cameras without wasting observations.

This project addresses that problem by predicting future target locations with Kalman filters, building a time-expanded network over candidate camera actions, assigning values to candidate observations, and solving a network flow optimization problem repeatedly online.

## Main ideas

The implementation includes the following components:

- **Kalman-filter-based target prediction**  
  Each tracked object is assigned a Kalman filter to predict future locations over the planning horizon.

- **Dynamic network flow scheduling**  
  Camera-task assignment is formulated as an optimization problem over a time-expanded network with camera nodes, track-location nodes, fixed-location nodes, demand nodes, and a sink node.

- **Fixed-location monitoring constraints**  
  The model enforces periodic surveillance of predefined regions to maintain coverage of the broader scene.

- **Value-based prioritization**  
  Candidate camera actions are scored using factors such as target urgency, track departure time, number of visible targets, and viewing geometry.

- **Group tracking via set cover**  
  Nearby predicted targets can be grouped into shared interrogation opportunities, reducing redundant tracking and improving camera utilization.

## Method summary

At a high level, the online loop works as follows:

1. Detect and track visible objects.
2. Update object states and Kalman filters.
3. Predict future target locations over a short horizon.
4. Generate candidate track and fixed-location nodes.
5. Build the network flow constraints and objective values.
6. Solve the resulting optimization problem.
7. Convert the optimal decision vector into camera job buffers.
8. Execute the next scheduled camera actions.
9. Repeat as new detections arrive.

## Code structure

The main decision rule is implemented in:

- `runNetworkFlowDecisionRule(...)`

Key functions include:

- `generatePredictedNodes(...)`  
  Builds predicted track nodes and fixed-location nodes over the planning horizon.

- `generateConstraints(...)`  
  Constructs the equality constraints, lower bounds, and upper bounds for the network flow problem.

- `generateValueVector(...)`  
  Computes the optimization objective values for camera-to-track and camera-to-fixed-location assignments.

- `controlCameras(...)`  
  Replans periodically, solves the optimization problem, and loads the resulting jobs into each camera.

- `generateSets(...)` / `runSetCoverGreedyAlgo(...)`  
  Build and prune group-tracking candidates using a greedy set cover strategy.

- `calculateMetrics(...)`  
  Evaluates watched-object ratio, average delay, and missed-object ratio.

Depending on how you organize the repository, you may also have supporting classes and utilities for:

- cameras,
- detected objects,
- tracks,
- Kalman filtering,
- synthetic scenario generation,
- visualization,
- and metric plotting.

## Problem setting

The simulated surveillance environment includes:

- multiple PTZ cameras,
- moving pedestrians entering according to a Poisson process,
- a 2D scene with no obstacles,
- wide-area and zoomed-in observation modes,
- constant task transition and focus times,
- and repeated replanning over a finite horizon.

The paper studies a representative setup with three PTZ cameras placed along one side of the scene, pedestrian motion modeled in a Cartesian plane, and state estimation based on noisy measurements of bounding-box geometry and position.

## Evaluation metrics

The main metrics used in the experiments are:

- **Watched ratio**: fraction of watchable objects successfully interrogated,
- **Average wait time**: average delay between object appearance and interrogation,
- **Missed-object ratio**: fraction of watchable objects that leave without interrogation.

The code also tracks additional efficiency-related quantities, such as:

- number of interrogations,
- inefficient interrogation ratio,
- swapped-track counts,
- and mean number of objects captured per interrogation.

## Reported results

In the paper, the proposed flexible PTZ scheduling system outperforms a conventional master-slave camera architecture. The group-tracking extension further improves performance by increasing coverage, reducing waiting time, and lowering the number of missed objects in crowded scenarios.

For the reported scenarios, the flexible system with group tracking achieved near-complete coverage and substantially lower missed-object ratios than the master-slave baseline.

## Requirements

This codebase is written in MATLAB.

You will likely need:

- MATLAB
- Optimization Toolbox (`linprog`)
- Computer Vision Toolbox or equivalent utilities for tracking and Kalman filtering

Depending on your local setup, you may also need custom class files used by the code, such as camera and detected-object classes.

## Running the code

A typical workflow is:

1. Prepare or load the simulated moving-object data.
2. Define watchable object IDs and simulation parameters.
3. Set the required global parameters, such as:
   - `focusTime`
   - `transitTime`
   - `frameRate`
   - `viewBox`
   - `vidXDim`
   - `vidYDim`
   - `measurementVars`
   - `processVars`
4. Call the main routine:
   ```matlab
   [watchedObjectsRatio, objectsMeanDelayTime, missedObjects, ...
       inefficientInterrogationRatio, interrogationCounter, swappedTrackCounter, ...
       meanOfInterrogatedObjectsPerInterrogation] = ...
       runNetworkFlowDecisionRule(movingObjects, listOfWatchableIds, ...
       numFrames, measurementVars, surveillanceFreq, overlapIntervalCheck);
