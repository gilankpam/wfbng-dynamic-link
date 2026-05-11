{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = with pkgs; [
    gnumake
    gcc
    pkgsCross.armv7l-hf-multiplatform.pkgsMusl.stdenv.cc
    (python3.withPackages (ps: with ps; [
      numpy
      plotly
      pyyaml
      pytest
      pytest-asyncio
    ]))
  ];
}