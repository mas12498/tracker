# tracker

Tracker is a library for simulating, transforming and fusing sensor measurements.

## Increments

 - **Increment I - III** : An interactive gui that can load a set of sensor pedestals and a set of targets. It allows the user to quickly calculate a number of quantities;
   - selecting a target points all the pedestals at the target, perturbed by their error model
   - Selecting a set of sensors generates a hypothetical fused solution for each target, and compares the expected error

 - **EnsembleTest** : 
  
 - **MonteCarloTest** : Produces a heatmap of accuracy for a given sensor ensemble using the montecarlo method.

 - **TestFilter** : Applies a filter to a simulated ensemble tracking a second order Trajectory.
   - Can also generate target files from the filter's output.

 - **TestMeasurments** : Applies a filter to pre-recorded measurements.
   - Can also simulate pre-recorded measurements using a Trajectory 

## Usage

``` console
\t Main
Project:
PredictiveTspi
Main class:
tspi.controller.EnsembleTest

\t (x)=Arguments:
-Solve
/home/mike/repos/mike/pedestalsExample.csv
/home/mike/repos/mike/ensembleSet.csv
/home/mike/repos/mike/solution.csv
4
true

----

Name: EnsembleTestGenerate

\t Main
Project:
PredictiveTspi
Main class:
tspi.controller.EnsembleTest

\t (x)=Arguments:
-Generate
/home/mike/repos/mike/pedestalsExample.csv
/home/mike/repos/mike/targetsExample.csv
/home/mike/repos/mike/ensembleSet.csv
```

### Helpful Links

[Workflow](https://nvie.com/posts/a-successful-git-branching-model/)

[Markdown Syntax](https://guides.github.com/features/mastering-markdown/)

[Eclipse EGit](http://wiki.eclipse.org/EGit/User_Guide#GitHub_Tutorial)
