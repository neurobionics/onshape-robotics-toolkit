from onshape_robotics_toolkit.native import OnshapeClient

# https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef

if __name__ == "__main__":
    print("\n=== Testing Rust Client ===")
    rs_client = OnshapeClient(
        env_file_path=".env",
    )

    rs_elements = rs_client.get_elements(
        "cf6b852d2c88d661ac2e17e8",
        "w",
        "c842455c29cc878dc48bdc68",
    )
    print(rs_elements["Tie"].id)

    print(f"\nAPI call count: {rs_client.api_call_count}")
