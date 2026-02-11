import pytest
from onshape_robotics_toolkit.models.assembly import (
    Assembly,
    AssemblyFeature,
    AssemblyFeatureType,
    MateConnectorFeatureData,
    MatedCS,
    Occurrence,
    PartInstance,
    InstanceType,
    RootAssembly,
    SubAssembly,
    AssemblyInstance,
    MateFeatureData,
    MateType
)
from onshape_robotics_toolkit.models.document import DocumentMetaData, DefaultWorkspace, MetaWorkspaceType, Document
from onshape_robotics_toolkit.parse import CAD

def test_populate_mate_connectors():
    part_id = "Mpart123"
    mc_id = "MC123"
    
    part_instance = PartInstance(
        id=part_id,
        name="Test Part",
        type=InstanceType.PART,
        suppressed=False,
        isStandardContent=False,
        partId="part1",
        fullConfiguration="default",
        configuration="default",
        documentId="a" * 24,
        elementId="b" * 24,
        documentMicroversion="c" * 24,
        documentVersion=None
    )
    
    occurrence = Occurrence(
        path=[part_id],
        transform=[1.0] + [0.0]*11 + [0.0, 0.0, 0.0, 1.0],
        fixed=False,
        hidden=False
    )
    
    mc_data = MateConnectorFeatureData(
        name="My Mate Connector",
        occurrence=[part_id],
        mateConnectorCS=MatedCS(
            xAxis=[1.0, 0.0, 0.0],
            yAxis=[0.0, 1.0, 0.0],
            zAxis=[0.0, 0.0, 1.0],
            origin=[0.1, 0.2, 0.3],
            tf=None
        ),
        id=mc_id
    )
    
    mc_feature = AssemblyFeature(
        id=mc_id,
        suppressed=False,
        featureType=AssemblyFeatureType.MATECONNECTOR,
        featureData=mc_data
    )
    
    assembly = Assembly(
        rootAssembly=RootAssembly(
            instances=[part_instance],
            occurrences=[occurrence],
            features=[mc_feature],
            patterns=[],
            fullConfiguration="default",
            configuration="default",
            documentId="a" * 24,
            elementId="b" * 24,
            documentMicroversion="c" * 24,
            documentMetaData=DocumentMetaData(
                defaultWorkspace=DefaultWorkspace(id="d" * 24, type=MetaWorkspaceType.WORKSPACE),
                name="test_doc",
                id="a" * 24
            ),
            MassProperty=None,
            isRigid=False,
            RootOccurrences=None
        ),
        subAssemblies=[],
        parts=[],
        partStudioFeatures=[],
        document=Document(
            did="a" * 24,
            wtype="w",
            wid="d" * 24,
            eid="e" * 24,
            name="test"
        ),
        name="test_assembly"
    )
    
    cad = CAD.from_assembly(assembly, max_depth=1, fetch_mate_properties=False)
    
    assert len(cad.mate_connectors) == 1
    assert cad.mate_connectors[0].name == "My Mate Connector"
    assert cad.mate_connectors[0].occurrence == [part_id]
    assert cad.mate_connectors[0].mateConnectorCS.origin == [0.1, 0.2, 0.3]

def test_populate_mate_connectors_in_subassembly():
    sub_inst_id = "Msub123"
    part_id = "Mpart456"
    mc_id = "MC456"
    
    sub_instance = AssemblyInstance(
        id=sub_inst_id,
        name="SubAssembly Instance",
        type=InstanceType.ASSEMBLY,
        suppressed=False,
        fullConfiguration="default",
        configuration="default",
        documentId="a" * 24,
        elementId="s" * 24,
        documentMicroversion="c" * 24,
        documentVersion=None,
        isRigid=False
    )
    
    part_instance = PartInstance(
        id=part_id,
        name="Nested Part",
        type=InstanceType.PART,
        suppressed=False,
        isStandardContent=False,
        partId="part2",
        fullConfiguration="default",
        configuration="default",
        documentId="a" * 24,
        elementId="s" * 24,
        documentMicroversion="c" * 24,
        documentVersion=None
    )
    
    sub_occ = Occurrence(
        path=[sub_inst_id],
        transform=[1.0] + [0.0]*11 + [0.0, 0.0, 0.0, 1.0],
        fixed=False,
        hidden=False
    )
    part_occ = Occurrence(
        path=[sub_inst_id, part_id],
        transform=[1.0] + [0.0]*11 + [0.0, 0.0, 0.0, 1.0],
        fixed=False,
        hidden=False
    )
    
    mc_data = MateConnectorFeatureData(
        name="Nested MC",
        occurrence=[part_id],
        mateConnectorCS=MatedCS(
            xAxis=[1.0, 0.0, 0.0],
            yAxis=[0.0, 1.0, 0.0],
            zAxis=[0.0, 0.0, 1.0],
            origin=[0.0, 0.0, 0.0],
            tf=None
        ),
        id=mc_id
    )
    
    mc_feature = AssemblyFeature(
        id=mc_id,
        suppressed=False,
        featureType=AssemblyFeatureType.MATECONNECTOR,
        featureData=mc_data
    )
    
    subassembly = SubAssembly(
        instances=[part_instance],
        features=[mc_feature],
        patterns=[],
        fullConfiguration="default",
        configuration="default",
        documentId="a" * 24,
        elementId="s" * 24,
        documentMicroversion="c" * 24,
        MassProperty=None,
        isRigid=False,
        RootOccurrences=None
    )
    
    root_assembly = RootAssembly(
        instances=[sub_instance],
        occurrences=[sub_occ, part_occ],
        features=[],
        patterns=[],
        fullConfiguration="default",
        configuration="default",
        documentId="a" * 24,
        elementId="r" * 24,
        documentMicroversion="c" * 24,
        documentMetaData=DocumentMetaData(
            defaultWorkspace=DefaultWorkspace(id="d" * 24, type=MetaWorkspaceType.WORKSPACE),
            name="root",
            id="a" * 24
        ),
        MassProperty=None,
        isRigid=False,
        RootOccurrences=None
    )
    
    assembly = Assembly(
        rootAssembly=root_assembly,
        subAssemblies=[subassembly],
        parts=[],
        partStudioFeatures=[],
        document=Document(
            did="a" * 24,
            wtype="w",
            wid="d" * 24,
            eid="r" * 24,
            name="test"
        ),
        name="test_assembly"
    )
    
    cad = CAD.from_assembly(assembly, max_depth=1, fetch_mate_properties=False)
    
    assert len(cad.mate_connectors) == 1
    assert cad.mate_connectors[0].name == "Nested MC"
    assert cad.mate_connectors[0].occurrence == [sub_inst_id, part_id]

def test_populate_mate_connectors_missing_pathkey():
    mc_id = "MC123"
    mc_data = MateConnectorFeatureData(
        name="Orphan MC",
        occurrence=["MissingPart"],
        mateConnectorCS=MatedCS(
            xAxis=[1.0, 0.0, 0.0],
            yAxis=[0.0, 1.0, 0.0],
            zAxis=[0.0, 0.0, 1.0],
            origin=[0.0, 0.0, 0.0],
            tf=None
        ),
        id=mc_id
    )
    mc_feature = AssemblyFeature(
        id=mc_id,
        suppressed=False,
        featureType=AssemblyFeatureType.MATECONNECTOR,
        featureData=mc_data
    )
    assembly = Assembly(
        rootAssembly=RootAssembly(
            instances=[],
            occurrences=[],
            features=[mc_feature],
            patterns=[],
            fullConfiguration="default",
            configuration="default",
            documentId="a" * 24,
            elementId="b" * 24,
            documentMicroversion="c" * 24,
            documentMetaData=DocumentMetaData(
                defaultWorkspace=DefaultWorkspace(id="d" * 24, type=MetaWorkspaceType.WORKSPACE),
                name="test_doc",
                id="a" * 24
            ),
            MassProperty=None,
            isRigid=False,
            RootOccurrences=None
        ),
        subAssemblies=[],
        parts=[],
        partStudioFeatures=[],
        document=Document(did="a" * 24, wtype="w", wid="d" * 24, eid="e" * 24, name="test"),
        name="test_assembly"
    )
    cad = CAD.from_assembly(assembly, max_depth=1, fetch_mate_properties=False)
    assert len(cad.mate_connectors) == 0

def test_populate_mate_connectors_wrong_type():
    bad_data = MateFeatureData(
        matedEntities=[],
        mateType=MateType.FASTENED,
        name="Bad MC",
        id="M1",
        limits=None
    )
    mc_feature = AssemblyFeature(
        id="M1",
        suppressed=False,
        featureType=AssemblyFeatureType.MATECONNECTOR,
        featureData=bad_data
    )
    assembly = Assembly(
        rootAssembly=RootAssembly(
            instances=[],
            occurrences=[],
            features=[mc_feature],
            patterns=[],
            fullConfiguration="default",
            configuration="default",
            documentId="a" * 24,
            elementId="b" * 24,
            documentMicroversion="c" * 24,
            documentMetaData=DocumentMetaData(
                defaultWorkspace=DefaultWorkspace(id="d" * 24, type=MetaWorkspaceType.WORKSPACE),
                name="test_doc",
                id="a" * 24
            ),
            MassProperty=None,
            isRigid=False,
            RootOccurrences=None
        ),
        subAssemblies=[],
        parts=[],
        partStudioFeatures=[],
        document=Document(did="a" * 24, wtype="w", wid="d" * 24, eid="e" * 24, name="test"),
        name="test_assembly"
    )
    cad = CAD.from_assembly(assembly, max_depth=1, fetch_mate_properties=False)
    assert len(cad.mate_connectors) == 0

def test_populate_mate_connectors_suppressed():
    part_id = "Mpart123"
    mc_id = "MC123"
    part_instance = PartInstance(
        id=part_id, name="Test Part", type=InstanceType.PART, suppressed=False, isStandardContent=False,
        partId="part1", fullConfiguration="default", configuration="default",
        documentId="a" * 24, elementId="b" * 24, documentMicroversion="c" * 24, documentVersion=None
    )
    occurrence = Occurrence(
        path=[part_id], transform=[1.0] + [0.0]*11 + [0.0, 0.0, 0.0, 1.0], fixed=False, hidden=False
    )
    mc_data = MateConnectorFeatureData(
        name="Suppressed MC", occurrence=[part_id],
        mateConnectorCS=MatedCS(xAxis=[1.0, 0.0, 0.0], yAxis=[0.0, 1.0, 0.0], zAxis=[0.0, 0.0, 1.0], origin=[0.0, 0.0, 0.0], tf=None),
        id=mc_id
    )
    mc_feature = AssemblyFeature(id=mc_id, suppressed=True, featureType=AssemblyFeatureType.MATECONNECTOR, featureData=mc_data)
    assembly = Assembly(
        rootAssembly=RootAssembly(
            instances=[part_instance], occurrences=[occurrence], features=[mc_feature], patterns=[],
            fullConfiguration="default", configuration="default", documentId="a" * 24, elementId="b" * 24,
            documentMicroversion="c" * 24, documentMetaData=DocumentMetaData(
                defaultWorkspace=DefaultWorkspace(id="d" * 24, type=MetaWorkspaceType.WORKSPACE), name="test_doc", id="a" * 24
            ), MassProperty=None, isRigid=False, RootOccurrences=None
        ), subAssemblies=[], parts=[], partStudioFeatures=[],
        document=Document(did="a" * 24, wtype="w", wid="d" * 24, eid="e" * 24, name="test"), name="test_assembly"
    )
    cad = CAD.from_assembly(assembly, max_depth=1, fetch_mate_properties=False)
    assert len(cad.mate_connectors) == 0

def test_populate_mate_groups():
    from onshape_robotics_toolkit.models.assembly import MateGroupFeatureData, MateGroupFeatureOccurrence
    mg_id = "MG123"
    mg_data = MateGroupFeatureData(
        name="My Mate Group",
        occurrences=[MateGroupFeatureOccurrence(occurrence=["part1"])],
        id=mg_id
    )
    mg_feature = AssemblyFeature(
        id=mg_id, suppressed=False, featureType=AssemblyFeatureType.MATEGROUP, featureData=mg_data
    )
    assembly = Assembly(
        rootAssembly=RootAssembly(
            instances=[], occurrences=[], features=[mg_feature], patterns=[],
            fullConfiguration="default", configuration="default", documentId="a" * 24, elementId="b" * 24,
            documentMicroversion="c" * 24, documentMetaData=DocumentMetaData(
                defaultWorkspace=DefaultWorkspace(id="d" * 24, type=MetaWorkspaceType.WORKSPACE), name="test_doc", id="a" * 24
            ), MassProperty=None, isRigid=False, RootOccurrences=None
        ), subAssemblies=[], parts=[], partStudioFeatures=[],
        document=Document(did="a" * 24, wtype="w", wid="d" * 24, eid="e" * 24, name="test"), name="test_assembly"
    )
    cad = CAD.from_assembly(assembly, max_depth=1, fetch_mate_properties=False)
    # Mate groups don't create edges or stay in CAD (they are processed and skipped)
    assert len(cad.mates) == 0
