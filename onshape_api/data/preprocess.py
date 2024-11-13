import json
import os
import re
from functools import partial

import numpy as np
import pandas as pd
from tqdm import tqdm

from onshape_api.connect import Client
from onshape_api.models import Assembly
from onshape_api.models.document import generate_url
from onshape_api.models.element import Element, ElementType
from onshape_api.utilities import LOGGER

AUTOMATE_ASSEMBLYID_PATTERN = r"(?P<documentId>\w{24})_(?P<documentMicroversion>\w{24})_(?P<elementId>\w{24})"


def extract_ids(assembly_id):
    match = re.match(AUTOMATE_ASSEMBLYID_PATTERN, assembly_id)
    if match:
        return match.groupdict()
    else:
        return {"documentId": None, "documentMicroversion": None, "elementId": None}


def raise_document_not_exist_error(documentId):
    raise ValueError(f"Document does not exist: {documentId}")


def get_assembly_data(assembly_id: str, client: Client):
    try:
        ids = extract_ids(assembly_id)
        document = client.get_document_metadata(ids["documentId"])

        if document is None:
            raise_document_not_exist_error(ids["documentId"])

        elements: list[Element] = client.get_elements(
            did=document.id, wtype=document.defaultWorkspace.type.shorthand, wid=document.defaultWorkspace.id
        )
        assembly_ids = [element.id for element in elements.values() if element.elementType == ElementType.ASSEMBLY]

        ids["elementId"] = assembly_ids
        ids["wtype"] = document.defaultWorkspace.type.shorthand
        ids["workspaceId"] = document.defaultWorkspace.id

        # LOGGER.info(f"Assembly data retrieved for element: {ids['elementId']}")

    except Exception as e:
        # LOGGER.warning(f"Error getting assembly data for {assembly_id}")
        LOGGER.warning(e)
        ids = {"documentId": None, "documentMicroversion": None, "elementId": None, "wtype": None, "workspaceId": None}

    return ids


def get_assembly_df_chunk(automate_assembly_df_chunk: pd.DataFrame, client: Client) -> pd.DataFrame:
    """
    Process a chunk of the automate assembly DataFrame.
    """
    _get_assembly_data = partial(get_assembly_data, client=client)
    assembly_df_chunk = automate_assembly_df_chunk["assemblyId"].progress_apply(_get_assembly_data).apply(pd.Series)
    return assembly_df_chunk


def get_assembly_df(automate_assembly_df: pd.DataFrame, client: Client, chunk_size: int = 1000) -> pd.DataFrame:
    """
    Process the automate assembly DataFrame in chunks and save checkpoints.
    """
    tqdm.pandas()
    total_rows = len(automate_assembly_df)
    chunks = (total_rows // chunk_size) + 1

    assembly_df_list = []
    try:
        for i in tqdm(range(chunks), desc="Processing chunks"):
            start_idx = i * chunk_size
            end_idx = min((i + 1) * chunk_size, total_rows)
            automate_assembly_df_chunk = automate_assembly_df.iloc[start_idx:end_idx]
            assembly_df_chunk = get_assembly_df_chunk(automate_assembly_df_chunk, client)
            assembly_df_list.append(assembly_df_chunk)
            checkpoint_path = f"assemblies_checkpoint_{i}.parquet"
            assembly_df_chunk.to_parquet(checkpoint_path, engine="pyarrow")

    except KeyboardInterrupt:
        LOGGER.warning("Processing interrupted. Saving progress...")

    assembly_df = pd.concat(assembly_df_list, ignore_index=True) if assembly_df_list else pd.DataFrame()

    return assembly_df


def process_all_checkpoints(client: Client):
    assemblies_df = pd.DataFrame()
    MAX_CHECKPOINTS = 256

    for i in range(MAX_CHECKPOINTS):
        checkpoint_path = f"assemblies_checkpoint_{i}.parquet"
        if os.path.exists(checkpoint_path):
            assembly_df = pd.read_parquet(checkpoint_path, engine="pyarrow")
            LOGGER.info(f"Processing checkpoint: {checkpoint_path} with {assembly_df.shape[0]} rows")
            assembly_df.dropna(subset=["documentId", "elementId"], inplace=True)

            assembly_df["elementId"] = assembly_df["elementId"].apply(
                lambda x: ", ".join(x) if isinstance(x, (list, np.ndarray)) else x
            )
            # drop all duplicate entries
            assembly_df.drop_duplicates(subset=["documentId", "elementId"], inplace=True)
            LOGGER.info(f"Checkpoint {checkpoint_path} processed with {assembly_df.shape[0]} rows")
            LOGGER.info("--" * 20)
            assemblies_df = pd.concat([assemblies_df, assembly_df], ignore_index=True)

    assemblies_df["elementId"] = assemblies_df["elementId"].apply(lambda x: x.split(", ") if isinstance(x, str) else x)

    # now for every elementId in the list, we will have a separate row
    assemblies_df = assemblies_df.explode("elementId")
    assemblies_df.to_parquet("assemblies.parquet", engine="pyarrow")


def save_all_jsons(client: Client):
    if not os.path.exists("assemblies.parquet"):
        # TODO: Re-process the automate data if assemblies.parquet is empty
        automate_assembly_df = pd.read_parquet("automate_assemblies.parquet", engine="pyarrow")
        assembly_df = get_assembly_df(automate_assembly_df, client=client)
        assembly_df.to_parquet("assemblies.parquet", engine="pyarrow")
    else:
        assembly_df = pd.read_parquet("assemblies.parquet", engine="pyarrow")

    json_dir = "json"
    os.makedirs(json_dir, exist_ok=True)

    for _, row in tqdm(assembly_df.iterrows(), total=len(assembly_df)):
        try:
            _, assembly_json = client.get_assembly(
                did=row["documentId"],
                wtype=row["wtype"],
                wid=row["workspaceId"],
                eid=row["elementId"],
                log_response=False,
            )

            json_file_path = os.path.join(json_dir, f"{row['documentId']}_{row["elementId"]}.json")
            with open(json_file_path, "w") as json_file:
                json.dump(assembly_json, json_file, indent=4)

            LOGGER.info(f"Assembly JSON saved to {json_file_path}")

        except Exception as e:
            LOGGER.warning(f"Error saving assembly JSON: {os.path.abspath(json_file_path)}")
            document_url = generate_url(row["documentId"], row["wtype"], row["workspaceId"], row["elementId"])
            LOGGER.warning(f"Onshape document: {document_url}")
            LOGGER.warning(f"Assembly JSON: {assembly_json}")
            LOGGER.warning(f"Element ID: {row['elementId']}")
            LOGGER.warning(e)
            continue


def validate_assembly_json(json_file_path: str):
    with open(json_file_path) as json_file:
        assembly_json = json.load(json_file)

    return Assembly.model_validate(assembly_json)


if __name__ == "__main__":
    client = Client()
    save_all_jsons(client)
