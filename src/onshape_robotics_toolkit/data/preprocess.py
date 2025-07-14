import json
import os
import re
from functools import partial

import numpy as np
import pandas as pd

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.models import Assembly
from onshape_robotics_toolkit.models.document import generate_url
from onshape_robotics_toolkit.models.element import Element, ElementType
from onshape_robotics_toolkit.utilities import LOGGER

AUTOMATE_ASSEMBLYID_PATTERN = (
    r"(?P<documentId>\w{24})_(?P<documentMicroversion>\w{24})_(?P<elementId>\w{24})"
)


def extract_ids(assembly_id: str) -> dict[str, str]:
    """
    Extract the documentId, documentMicroversion, and elementId from the assemblyId.

    Args:
        assembly_id: The assemblyId to extract the ids from.

    Returns:
        A dictionary with the documentId, documentMicroversion, and elementId.
    """
    match = re.match(AUTOMATE_ASSEMBLYID_PATTERN, assembly_id)
    if match:
        return match.groupdict()
    else:
        return {"documentId": None, "documentMicroversion": None, "elementId": None}


def raise_document_not_exist_error(documentId: str):
    """
    Raise an error if the document does not exist.

    Args:
        documentId: The id of the document.
    """
    raise ValueError(f"Document does not exist: {documentId}")


def get_assembly_data(assembly_id: str, client: Client) -> dict[str, str]:
    """
    Get the assembly data from the assemblyId.

    Args:
        assembly_id: The assemblyId to get the data from.
        client: The client to use to get the data.

    Returns:
        A dictionary with the assembly data.
    """
    try:
        ids = extract_ids(assembly_id)
        document = client.get_document_metadata(ids["documentId"])

        if document is None:
            raise_document_not_exist_error(ids["documentId"])

        elements: list[Element] = client.get_elements(
            did=document.id,
            wtype=document.defaultWorkspace.type.shorthand,
            wid=document.defaultWorkspace.id,
        )
        assembly_ids = [
            element.id
            for element in elements.values()
            if element.elementType == ElementType.ASSEMBLY
        ]

        ids["elementId"] = assembly_ids
        ids["wtype"] = document.defaultWorkspace.type.shorthand
        ids["workspaceId"] = document.defaultWorkspace.id

        # LOGGER.info(f"Assembly data retrieved for element: {ids['elementId']}")

    except Exception as e:
        # LOGGER.warning(f"Error getting assembly data for {assembly_id}")
        LOGGER.warning(e)
        ids = {
            "documentId": None,
            "documentMicroversion": None,
            "elementId": None,
            "wtype": None,
            "workspaceId": None,
        }

    return ids


def get_assembly_df_chunk(
    automate_assembly_df_chunk: pd.DataFrame, client: Client
) -> pd.DataFrame:
    """
    Process a chunk of the automate assembly DataFrame.

    Args:
        automate_assembly_df_chunk: The chunk of the automate assembly DataFrame to process.
        client: The client to use to get the data.

    Returns:
        A DataFrame with the assembly data.
    """
    _get_assembly_data = partial(get_assembly_data, client=client)
    assembly_df_chunk = (
        automate_assembly_df_chunk["assemblyId"]
        .progress_apply(_get_assembly_data)
        .apply(pd.Series)
    )
    return assembly_df_chunk


def get_assembly_df(
    automate_assembly_df: pd.DataFrame, client: Client, chunk_size: int = 1000
) -> pd.DataFrame:
    """
    Process the automate assembly DataFrame in chunks and save checkpoints.

    Args:
        automate_assembly_df: The automate assembly DataFrame to process.
        client: The client to use to get the data.
        chunk_size: The size of the chunks to process.

    Returns:
        A DataFrame with the assembly data.
    """
    total_rows = len(automate_assembly_df)
    chunks = (total_rows // chunk_size) + 1

    assembly_df_list = []
    try:
        for i in range(chunks):
            start_idx = i * chunk_size
            end_idx = min((i + 1) * chunk_size, total_rows)
            automate_assembly_df_chunk = automate_assembly_df.iloc[start_idx:end_idx]
            assembly_df_chunk = get_assembly_df_chunk(
                automate_assembly_df_chunk, client
            )
            assembly_df_list.append(assembly_df_chunk)
            checkpoint_path = f"assemblies_checkpoint_{i}.parquet"
            assembly_df_chunk.to_parquet(checkpoint_path, engine="pyarrow")

    except KeyboardInterrupt:
        LOGGER.warning("Processing interrupted. Saving progress...")

    assembly_df = (
        pd.concat(assembly_df_list, ignore_index=True)
        if assembly_df_list
        else pd.DataFrame()
    )

    return assembly_df


def process_all_checkpoints() -> pd.DataFrame:
    """
    Process all the checkpoints and return the final DataFrame.

    Returns:
        A DataFrame with the assembly data.
    """
    assemblies_df = pd.DataFrame()
    MAX_CHECKPOINTS = 256

    for i in range(MAX_CHECKPOINTS):
        checkpoint_path = f"assemblies_checkpoint_{i}.parquet"
        if os.path.exists(checkpoint_path):
            assembly_df = pd.read_parquet(checkpoint_path, engine="pyarrow")
            LOGGER.info(
                f"Processing checkpoint: {checkpoint_path} with {assembly_df.shape[0]} rows"
            )
            assembly_df.dropna(subset=["documentId", "elementId"], inplace=True)

            assembly_df["elementId"] = assembly_df["elementId"].apply(
                lambda x: ", ".join(x) if isinstance(x, list | np.ndarray) else x
            )
            # drop all duplicate entries
            assembly_df.drop_duplicates(
                subset=["documentId", "elementId"], inplace=True
            )
            LOGGER.info(
                f"Checkpoint {checkpoint_path} processed with {assembly_df.shape[0]} rows"
            )
            LOGGER.info("--" * 20)
            assemblies_df = pd.concat([assemblies_df, assembly_df], ignore_index=True)

    assemblies_df["elementId"] = assemblies_df["elementId"].apply(
        lambda x: x.split(", ") if isinstance(x, str) else x
    )

    # now for every elementId in the list, we will have a separate row
    assemblies_df = assemblies_df.explode("elementId")
    assembly_df["url"] = assembly_df.apply(get_assembly_url, axis=1)
    assemblies_df.to_parquet("assemblies.parquet", engine="pyarrow")


def validate_assembly_json(json_file_path: str) -> Assembly:
    """
    Validate the assembly JSON file.

    Args:
        json_file_path: The path to the assembly JSON file.

    Returns:
        The validated assembly.
    """
    with open(json_file_path) as json_file:
        assembly_json = json.load(json_file)

    return Assembly.model_validate(assembly_json)


def get_assembly_url(row: pd.Series) -> str:
    """
    Get the assembly URL from the row.

    Args:
        row: The row to get the URL from.

    Returns:
        The assembly URL.
    """
    return generate_url(
        row["documentId"], row["wtype"], row["workspaceId"], row["elementId"]
    )


def get_automate_assembly_df(
    path: str = "automate_assemblies.parquet",
) -> pd.DataFrame | None:
    """
    Get the automate assembly DataFrame.

    Args:
        path: The path to the automate assembly DataFrame.

    Returns:
        The automate assembly DataFrame.
    """
    if os.path.exists(path):
        automate_assembly_df = pd.read_parquet(
            "automate_assemblies.parquet", engine="pyarrow"
        )
    else:
        LOGGER.warning(
            "Download automate dataset from here: https://zenodo.org/records/7776208/files/assemblies.parquet?download=1"
        )
        automate_assembly_df = None

    return automate_assembly_df


if __name__ == "__main__":
    client = Client()

    try:
        assembly_df = pd.read_parquet("assemblies.parquet", engine="pyarrow")

    except FileNotFoundError:
        LOGGER.warning("assemblies.parquet not found, looking for automate dataset...")
        automate_assembly_df = get_automate_assembly_df()

        if automate_assembly_df:
            assembly_df = get_assembly_df(automate_assembly_df, client)
            assembly_df.to_parquet("assemblies.parquet", engine="pyarrow")

        else:
            LOGGER.warning("Automate dataset not found. Exiting...")
            exit()

    LOGGER.info(assembly_df.head())
    LOGGER.info(assembly_df.shape)
