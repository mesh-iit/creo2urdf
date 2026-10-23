# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

- Export `model.json` in `idyntree-model-json` format alongside `model.urdf`, using a JSON-enabled iDynTree 15.0.1 vcpkg overlay.
- Always normalize link frames for URDF export, expanding spherical joints before frame relocation.
- Added `includes` parameter to `creo2urdf` to include additional yamls.

## [0.4.7] - 2024-04-09
- Made `creo2urdf` runnable from terminal

## [0.4.6] - 2024-03-19
- Account for part/link rotation when saving inertia matrix

## [0.4.5] - 2024-01-17

- Added doxygen documentation
- Removed the assumptions on the matching of axis and csys for defining joints.
- Added the possibility to choose the output folder.
- Added the possibility to choose the mesh format.
- Added the possibility to export prismatic joints.

## [0.4.1] - 2023-11-23

- Added `warningsAreFatal` parameter
- Added handling of exception when exporting stl files
- Fixed urdf identation

## [0.4.0] - 2023-09-01

- `creo2urdf` can export the urdf of an entire generic robot

## [0.3.0] - 2023-07-20

- `creo2urdf` can export the urdf of the entire ergocub 1.1 

## [0.2.5] - 2023-07-05

- `creo2urdf` can export the urdf of ergocub upperbody

## [0.2.0] - 2023-06-30

- `creo2urdf` can now export the urdf of the ergocub head+neck

## [0.1.0] - 2023-05-10

- First version of `creo2urdf` plugin working with the simple assembly with two links and one joint.

