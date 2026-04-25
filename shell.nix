{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = with pkgs; [
    gnumake
    gcc
    pkgsCross.armv7l-hf-multiplatform.pkgsMusl.stdenv.cc
  ];
}