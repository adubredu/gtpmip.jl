# gtpmip
## Grounded Task Planning as Mixed Integer Programming.
Julia package for the `GTPMIP` algorithm. Given an optimal constrained task planning problem specified in an [hpd](https://github.com/adubredu/HPD.jl) file, GTPMIP returns a logically coherent sequence of actions, each with its corresponding optimal continuous parameters.

## Installation
1. Open your Julia REPL by typing  `julia` in your terminal.
2. Press `]` on your keyboard to enter the package manager
3. Enter command `add https://github.com/adubredu/gtpmip.jl` and press 
`Enter` on your keyboard to install this package.
4. Press the `Backspace` key on your keyboard to return to the REPL

This package uses Gurobi to solve Mixed Integer Programs. As such, a Gurobi license is required to use this package. Instructions on how to acquire a free academic Gurobi license can be found [here](https://www.gurobi.com/academia/academic-program-and-licenses/)

## Usage
Example usage scripts can be found in the [experiments](experiments) folder.

## Webpage
More information about this algorithm can be found on the [project webpage](https://adubredu.github.io/gtpmip)