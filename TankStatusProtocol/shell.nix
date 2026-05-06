{ pkgs ? import <nixpkgs> {} }:
pkgs.mkShell {
  packages = with pkgs; [
    cmake
    gcc
    python3
    python3Packages.pybind11
    python3Packages.pyserial   # you'll want this for the Mega↔Jetson bridge
  ];
}
