{
  description = "DevShell with ARM GCC 7-2018-q2 from old nixpkgs for VESC firmware development";

  inputs = {
    # Current stable nixpkgs
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";

    # Old nixpkgs containing gcc-arm-embedded-7 = 7-2018-q2-update
    nixpkgs-18_09 = {
      url = "github:NixOS/nixpkgs/18.09";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, nixpkgs-18_09, ... }:
  let
    system = "x86_64-linux";

    pkgs = import nixpkgs {
      inherit system;
    };

    # Import old nixpkgs as a regular package set
    pkgsOld = import nixpkgs-18_09 {
      inherit system;
    };

    # Try gcc-arm-embedded-7 first, fall back to gcc-arm-embedded if unavailable
    armToolchain =
      if pkgsOld ? gcc-arm-embedded-7 then pkgsOld.gcc-arm-embedded-7
      else pkgsOld.gcc-arm-embedded;
  in {
    devShells.${system}.default = pkgs.mkShell {
      buildInputs = [
        armToolchain
        pkgs.gnumake
        pkgs.git
        pkgs.openocd
      ];

      shellHook = ''
        echo "ARM toolchain from old nixpkgs:"
        arm-none-eabi-gcc --version || true
        echo
        echo "Ready to build! Example: make str500i USE_LISPBM=0"
      '';
    };
  };
}

