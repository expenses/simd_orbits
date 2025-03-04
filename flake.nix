{
  description = "Flake utils demo";

  inputs.flake-utils.url = "github:numtide/flake-utils";
  inputs.crane.url = "github:ipetkov/crane";
  inputs.fenix = {
    url = "github:nix-community/fenix";
    inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = {
    self,
    nixpkgs,
    flake-utils,
    crane,
    fenix,
  }:
    flake-utils.lib.eachDefaultSystem (
      system: let
        pkgs = nixpkgs.legacyPackages.${system};
        craneLib = (crane.mkLib pkgs).overrideToolchain (with fenix.packages.${system};
          combine [
            latest.rustc
            latest.cargo
            targets.wasm32-unknown-unknown.latest.rust-std
          ]);
      in {
        devShells.default = with pkgs;
          mkShell rec {
            nativeBuildInputs = [
              pkg-config
              clang
              linuxPackages_latest.perf
              hotspot
            ];
            buildInputs = [
              udev
              alsa-lib
              vulkan-loader
              xorg.libX11
              xorg.libXcursor
              xorg.libXi
              xorg.libXrandr # To use the x11 feature
              libxkbcommon
              wayland # To use the wayland feature
              stdenv.cc.cc
              simde
              ispc
              libclang.lib
            ];
            LD_LIBRARY_PATH = lib.makeLibraryPath buildInputs;
          };

          packages = rec {
            wasm = craneLib.buildPackage rec {
            name = "bev";
            src = craneLib.cleanCargoSource ./.;
            cargoArtifacts = craneLib.buildDepsOnly {
              name = "bev";
              inherit src CARGO_BUILD_TARGET;
            };
            CARGO_BUILD_TARGET = "wasm32-unknown-unknown";
            cargoExtraArgs = "--example=bev";
            doCheck = false;
          };

          default = with pkgs;
            runCommand "wasm-dir" {} ''
              echo ${wasm-bindgen-cli}/bin/wasm-bindgen ${wasm}/bin/bev.wasm --no-typescript --target web --out-dir $out
              ${wasm-bindgen-cli}/bin/wasm-bindgen ${wasm}/bin/bev.wasm --no-typescript --target web --out-dir $out
              echo ${binaryen}/bin/wasm-opt -Oz -o $out/bev_bg.wasm $out/bev_bg.wasm
              ${binaryen}/bin/wasm-opt -Oz -o $out/bev_bg.wasm $out/bev_bg.wasm
              cp ${./index.html} $out/index.html
              cp -r ${./assets} $out/assets
            '';
          };
      }
    );
}
