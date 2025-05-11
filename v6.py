from abaqus import *
from abaqusConstants import *
import section
import regionToolset
import displayGroupMdbToolset as dgm
import part
import material
import assembly
import step
import interaction
import load
import mesh
import optimization
import job
import sketch
import visualization
import xyPlot
import displayGroupOdbToolset as dgo
import connectorBehavior
import numpy as np
from scipy.interpolate import lagrange

#Geometrical informations
R_SI = 75
Y_SI = 100
R0_CU = np.array([15, 25, 10])
Y0_CU = np.array([0, 60, Y_SI])


#-------------------------------------------------
#Units are in MMKS(mm, N, ton, MPa, mJ, ton/mm^3)
#Material Properties(Cu)
E_CU = 120e3 # Elastic Modulus
NU_CU = 0.34 # Poisson's Ratio
CTE_CU = 17e-6
Y_CU = 150
DEN_CU = 8960e-12 #Density
CON_CU= 401e-3 #Thermal Conductivity
SH_CU = 390e6 #Specific heat

#Material Properties(Si)
E_SI = 130e3
NU_SI = 0.28
CTE_SI = 2.8e-6
K_CU = 0.0015e-3
C_CU = 700e6
RHO_CU = 2330e-12
CON_SI = 149e-3
DEN_SI = 2330000e-12
CTE_SI = 2.8e-06
SH_SI = 700.0e6

def s_t_cu(e_t_cu): #Plasticity true stress-strain curve
    if e_t_cu <= 0.02:
        return 8.0e4 * e_t_cu +0.001
    else:
        return 69.6 * (e_t_cu ** 0.286)
P_TABLE_CU = tuple((s_t_cu(e_t_cu), e_t_cu) for e_t_cu in np.linspace(0., 0.2, 200, endpoint=True).tolist())




#Cohesive Contact

#-------------------------------------------------
#r-z relation
r_CU = lagrange(Y0_CU, R0_CU)
ry_CU = [(r_CU(y), y) for y in np.linspace(0, Y_SI, 100, endpoint=True).tolist()]
#-------------------------------------------------
# 모델 생성
mymodel = mdb.models['Model-1']

cu_sketch = mymodel.ConstrainedSketch(name='revolve_profile', sheetSize=200.0)

# 회전시킬 CU 반평면 제작
v1 = cu_sketch.Spline(ry_CU)
v2 = cu_sketch.Line(point1 = ry_CU[-1], point2 = (0, Y_SI))
v3 = cu_sketch.Line(point1 = (0, Y_SI), point2 = (0, 0))
v4 = cu_sketch.Line(point1 = (0, 0), point2 = ry_CU[0])
#mysketch.CoincidentConstraint(v1, v2)

rotation_axis = cu_sketch.ConstructionLine(point1=(0, 0), point2=(0, Y_SI))
cu_sketch.assignCenterline(rotation_axis)



# Cu 파트 생성 (회전 축은 Y축)
cu_part = mymodel.Part(name='Cu',
                  dimensionality=THREE_D,
                  type=DEFORMABLE_BODY)

# 직사각형을 Y축을 기준으로 360도 회전하여 원통 형태로 만들기
cu_part.BaseSolidRevolve(sketch=cu_sketch, angle=360.0)


# Si 단면 생성
si_sketch = mymodel.ConstrainedSketch(name='Box_profile', sheetSize=200.0)
si_sketch.rectangle(point1=(-R_SI, 0), point2=(R_SI, Y_SI))  # 정사각형 단면

uncut_si_part = mymodel.Part(name='UncutSi', dimensionality=THREE_D, type=DEFORMABLE_BODY)
uncut_si_part.BaseSolidExtrude(sketch=si_sketch, depth=2*R_SI)  # 깊이 방향은 Y축

myassembly = mymodel.rootAssembly
cu_instance = myassembly.Instance(name='Cu', part=cu_part, dependent=ON)
uncut_si_instance = myassembly.Instance(name='UncutSi', part=uncut_si_part, dependent=ON)
myassembly.translate(instanceList=('UncutSi',), vector=(0,0,-R_SI))

si_part = myassembly.PartFromBooleanCut(
    name='Si',
    instanceToBeCut=uncut_si_instance,
    cuttingInstances=(cu_instance,),
    originalInstances=DELETE
)
del mymodel.parts['UncutSi']


##Material Properties
cu_material = mymodel.Material(name='Cu')
cu_material.Density(table=((DEN_CU, ), ))
cu_material.Conductivity(table=((CON_CU, ), ))
cu_material.Expansion(table=((CTE_CU, ), ))
cu_material.SpecificHeat(table=((SH_CU, ), ))
cu_material.Elastic(table = ((E_CU, NU_CU),)) #E, Poisson's ratio
cu_material.Plastic(table = P_TABLE_CU)


si_material = mymodel.Material(name = 'Si')
si_material.Elastic(table = ((E_SI, NU_SI),))
si_material.Conductivity(table=((CON_SI, ), ))
si_material.Density(table=((DEN_SI, ), ))
si_material.Expansion(table=((CTE_SI, ), ))
si_material.SpecificHeat(table=((SH_SI, ), ))

#Section & Section assignment
mymodel.HomogeneousSolidSection(name='Cu', material='Cu', thickness=None)
mymodel.HomogeneousSolidSection(name='Si', material='Si', thickness=None)


c = cu_part.cells
cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
region = cu_part.Set(cells=cells, name='Set-1')
cu_part.SectionAssignment(region=region, sectionName='Cu', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

p1 = si_part
c = p1.cells
cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
region = p1.Set(cells=cells, name='Set-1')
p1.SectionAssignment(region=region, sectionName='Si', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

#Assembly
a2 = mymodel.rootAssembly
a2.DatumCsysByDefault(CARTESIAN)
p = mymodel.parts['Cu']
a2.Instance(name='Cu-1', part=p, dependent=OFF)
p = mymodel.parts['Si']
a2.Instance(name='Si-1', part=p, dependent=OFF)

#Mesh
a1 = mymodel.rootAssembly
c1 = a1.instances['Cu-1'].cells
cells1 = c1.getSequenceFromMask(mask=('[#1 ]', ), )
pickedRegions =tuple(cells1, )
a1.setMeshControls(regions=pickedRegions, elemShape=WEDGE)
elemType1 = mesh.ElemType(elemCode=C3D8RT, elemLibrary=EXPLICIT, kinematicSplit=AVERAGE_STRAIN, secondOrderAccuracy=ON, hourglassControl=STIFFNESS, distortionControl=DEFAULT, elemDeletion=DEFAULT)
elemType2 = mesh.ElemType(elemCode=C3D6T, elemLibrary=EXPLICIT, secondOrderAccuracy=ON, distortionControl=DEFAULT, elemDeletion=ON)
a1.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2))

a1 = mymodel.rootAssembly
c1 = a1.instances['Si-1'].cells
cells1 = c1.getSequenceFromMask(mask=('[#1 ]', ), )
pickedRegions =tuple(cells1, )
a1.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2))
    
a1 = mymodel.rootAssembly
partInstances =(a1.instances['Cu-1'], a1.instances['Si-1'], )
a1.seedPartInstance(regions=partInstances, size=18.0, deviationFactor=0.2,minSizeFactor=0.1)
a1.generateMesh(regions=partInstances)





#Amplitude
mymodel.PeriodicAmplitude(name='Amp-1', timeSpan=STEP,  frequency=0.5, start=0.0, a_0=1.0, data=((0.0, 1.0), ))

#BC
a1 = mymodel.rootAssembly
f1 = a1.instances['Si-1'].faces
faces1 = f1.getSequenceFromMask(mask=('[#20 ]', ), )
region = a1.Set(faces=faces1, name='Set-6')
mymodel.YsymmBC(name='BC-1', createStepName='Initial',region=region, localCsys=None)
faces1 = f1.getSequenceFromMask(mask=('[#8 ]', ), )
region = a1.Set(faces=faces1, name='Set-7')
mymodel.YsymmBC(name='BC-2', createStepName='Initial', region=region, localCsys=None)
mymodel.TempDisplacementDynamicsStep(name='Step-1', previous='Initial', timePeriod=20.0, improvedDtMethod=ON)
a1 = mymodel.rootAssembly
c1 = a1.instances['Cu-1'].cells
cells1 = c1.getSequenceFromMask(mask=('[#1 ]', ), )
f1 = a1.instances['Cu-1'].faces
faces1 = f1.getSequenceFromMask(mask=('[#7 ]', ), )
e1 = a1.instances['Cu-1'].edges
edges1 = e1.getSequenceFromMask(mask=('[#7 ]', ), )
v1 = a1.instances['Cu-1'].vertices
verts1 = v1.getSequenceFromMask(mask=('[#3 ]', ), )
c2 = a1.instances['Si-1'].cells
cells2 = c2.getSequenceFromMask(mask=('[#1 ]', ), )
f2 = a1.instances['Si-1'].faces
faces2 = f2.getSequenceFromMask(mask=('[#7f ]', ), )
e2 = a1.instances['Si-1'].edges
edges2 = e2.getSequenceFromMask(mask=('[#7fff ]', ), )
v2 = a1.instances['Si-1'].vertices
verts2 = v2.getSequenceFromMask(mask=('[#2ef ]', ), )
region = a1.Set(vertices=verts1+verts2, edges=edges1+edges2, faces=faces1+faces2, cells=cells1+cells2, name='Set-8')
mymodel.TemperatureBC(name='BC-3', createStepName='Step-1', region=region, fixed=OFF, distributionType=UNIFORM, fieldName='', magnitude=50.0, amplitude='Amp-1')


#Job

mdb.Job(name='mymodel', model=mymodel, description='', type=ANALYSIS, 
        atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
        memoryUnits=PERCENTAGE, explicitPrecision=SINGLE, 
        nodalOutputPrecision=SINGLE, echoPrint=OFF, modelPrint=OFF, 
        contactPrint=OFF, historyPrint=OFF, userSubroutine='', scratch='', 
        resultsFormat=ODB)
mdb.jobs['mymodel'].submit(consistencyChecking=OFF)




