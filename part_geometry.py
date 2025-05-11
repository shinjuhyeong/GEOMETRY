from abaqus import *
from abaqusConstants import *
import numpy as np
from scipy.interpolate import lagrange
from mesh import ElemType
import locale
import step
import interaction
from regionToolset import Region

locale.setlocale(locale.LC_ALL, 'en_US.UTF-8')
#-------------------------------------------------
#Units are in MMKS(mm, kg, s, kg/m^3, Pa)
#Geometrical informations, Geometry is scaled up to 1000 times
R_SI = 100
Y_SI = 100
R0_CU = np.array([15, 15, 10])
Y0_CU = np.array([0, 60, Y_SI])
#-------------------------------------------------
#Material Properties(Cu)
E_CU = 120e3 # Elastic Modulus
NU_CU = 0.34 # Poisson's Ratio
CTE_CU = 17e-6
Y_CU = 150
e_t_values = np.linspace(0, 0.2, 200, endpoint=True).tolist()
P_TABLE_CU = [(140 + 69.6 * (e_t_cu ** 0.286), e_t_cu) for e_t_cu in e_t_values]
K_CU = 0.401 #Thermal Conductivity
C_CU = 390e-6 #Specific heat
RHO_CU = 8.960e-12 #Density

# Material Properties (Si - Silicon) in MMKS
E_SI = 130000  # Elastic Modulus in MPa
NU_SI = 0.28  # Poisson's Ratio
CTE_SI = 2.8e-6  # Coefficient of Thermal Expansion in 1/°C
K_SI = 0.149  # Thermal Conductivity in W/mm·K (converted from W/m·K)
C_SI = 700e-6  # Specific Heat in mJ/kg·K (converted from J/kg·K)
RHO_SI = 2.33e-12  # Density in kg/mm³ (converted from kg/m³)

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

#myassembly에 assembly된 부품을 삭제하고, 새로운 부품을 assembly에 추가. 최종적인 assembly은 Cu와 Si로 구성됨
cu_instance = myassembly.Instance(name='Cu', part=cu_part, dependent=ON)
si_instance = myassembly.Instance(name='Si', part=si_part, dependent=ON)


# Calculate the midpoint of the ry_CU list
mid_index = len(ry_CU) // 2
mid_point = ry_CU[mid_index]
# 1. ContactSurf : Define contact surfaces for CU and SI parts
contactsurf_cu = cu_part.Surface(name='ContactSurfCu', side1Faces=cu_part.faces.findAt(((mid_point[0], mid_point[1], 0),)))
contactsurf_si = si_part.Surface(name='ContactSurfSi', side1Faces=si_part.faces.findAt(((mid_point[0], mid_point[1], 0),)))

# 2. Volume : Create sets for all cells in Si and Cu parts
si_part.Set(name='Sivolume', cells=si_part.cells[:])
cu_part.Set(name='Cuvolume', cells=cu_part.cells[:])

# Manually select the four lateral faces of Si
left_face = si_part.faces.findAt(((-R_SI, Y_SI / 2, 0),))  # Left face
right_face = si_part.faces.findAt(((R_SI, Y_SI / 2, 0),))  # Right face
front_face = si_part.faces.findAt(((0, Y_SI / 2, -R_SI),))  # Front face
back_face = si_part.faces.findAt(((0, Y_SI / 2, R_SI),))  # Back face
# 3. SiLateralSurf : Combine the selected faces into a single set# Combine the selected faces into a single set
si_part.Set(name='SiLateralSurf', faces=(left_face, right_face, front_face, back_face))


#-------------------------------------------------
# 구리 재료 정의
cu_material = mymodel.Material(name = 'Cu')
cu_material.Elastic(table=((E_CU, NU_CU),)) # Elastic Modulus and Poisson's Ratio
cu_material.Plastic(table=P_TABLE_CU) # Plasticity true stress-strain curve
cu_material.Expansion(table=((CTE_CU,),)) # Coefficient of Thermal Expansion
cu_material.Density(table=((RHO_CU,),)) # Density
cu_material.Conductivity(table=((K_CU,),)) # Thermal Conductivity
cu_material.SpecificHeat(table=((C_CU,),)) # Specific Heat

# 실리콘 재료 정의
si_material = mymodel.Material(name='Si')
si_material.Elastic(table=((E_SI, NU_SI),)) # Elastic Modulus and Poisson's Ratio
si_material.Expansion(table=((CTE_SI,),)) # Coefficient of Thermal Expansion
si_material.Density(table=((RHO_SI,),)) # Density
si_material.Conductivity(table=((K_SI,),)) # Thermal Conductivity
si_material.SpecificHeat(table=((C_SI,),)) # Specific Heat
#-------------------------------------------------

# Define Via section
via_section = mymodel.HomogeneousSolidSection(name='Via', material='Cu', thickness=None)

# Define Wafer section
wafer_section = mymodel.HomogeneousSolidSection(name='wafer', material='Si', thickness=None)

# Assign sections to parts
cu_region = (cu_part.cells, )
cu_part.SectionAssignment(region=cu_region, sectionName='Via')

si_region = (si_part.cells, )
si_part.SectionAssignment(region=si_region, sectionName='wafer')

myassembly.regenerate() # 할 필요는 없지만 안전을 위해 assembly 최신화

#-------------------------------------------------
#Steps
# 사용자 정의 Amplitude 생성 (온도 사이클)
single_cycle_data = [
    (0.0, 25.0),       # 시간 0분, 온도 25°C
    (18.33, 300.0),    # 15도/분으로 가열하여 18.33분 후 300°C 도달
    (28.33, 300.0),    # 10분간 300°C 유지
    (44.33, -65.0),    # -10도/분으로 냉각하여 44.33분 후 -65°C 도달
    (54.33, -65.0),    # 10분간 -65°C 유지
    (70.0, 25.0)       # 15도/분으로 가열하여 70분 후 25°C 도달
]

single_cycle_data_multi = [
    (18.33, 300.0),    # 15도/분으로 가열하여 18.33분 후 300°C 도달
    (28.33, 300.0),    # 10분간 300°C 유지
    (44.33, -65.0),    # -10도/분으로 냉각하여 44.33분 후 -65°C 도달
    (54.33, -65.0),    # 10분간 -65°C 유지
    (70.0, 25.0)       # 15도/분으로 가열하여 70분 후 25°C 도달
]

# 열사이클 반복 설정
total_cycles = 10
cycle_time = 70.0
custom_amplitude_data = single_cycle_data
for i in range(total_cycles-1):
    for time, temp in single_cycle_data_multi:
        custom_amplitude_data.append((time + (i+1) * cycle_time, temp))


# Amplitude 정의
mymodel.TabularAmplitude(name='ThermalCycle', timeSpan=TOTAL, smooth=SOLVER_DEFAULT, data=custom_amplitude_data)

# Step 설정
total_time = total_cycles * cycle_time

mymodel.TempDisplacementDynamicsStep(name='TCTCondition', previous='Initial', timePeriod=total_time, timeIncrementationMethod=FIXED_USER_DEFINED_INC, userDefinedInc=1e-4)

# Apply mass scaling to increase stable time increment
mymodel.massScaling(regionType=MODEL, massScalingType=SEMI_AUTOMATIC, frequency=0, factor=1e-4, region=None, stepName='TCTCondition')
#-------------------------------------------------
# Update Temperature BCs to reference sets in instances
region_sivolume = myassembly.instances['Si'].sets['Sivolume']
region_cuvolume = myassembly.instances['Cu'].sets['Cuvolume']

mymodel.TemperatureBC(name='SiTempBC', createStepName='TCTCondition', region=region_sivolume, 
                      distributionType=UNIFORM, fieldName='', magnitude=0.0, amplitude='ThermalCycle')

mymodel.TemperatureBC(name='CuTempBC', createStepName='TCTCondition', region=region_cuvolume, 
                      distributionType=UNIFORM, fieldName='', magnitude=0.0, amplitude='ThermalCycle')

# Convert the set to a region
region_silateralsurf = myassembly.instances['Si'].sets['SiLateralSurf']

# Add boundary condition to fix the lateral surfaces
mymodel.DisplacementBC(name='FixedSiLateral', createStepName='Initial', region=region_silateralsurf, 
                       u1=0.0, u2=0.0, u3=0.0, 
                       ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                       amplitude=UNSET, fixed=ON, distributionType=UNIFORM, fieldName='')



# Update the CuSiTie constraint to use instance contact surfaces
contactsurf_cu_instance = myassembly.instances['Cu'].surfaces['ContactSurfCu']
contactsurf_si_instance = myassembly.instances['Si'].surfaces['ContactSurfSi']

mymodel.Tie(name='CuSiTie', 
    main=contactsurf_cu_instance, 
    secondary=contactsurf_si_instance, 
    positionToleranceMethod=COMPUTED, 
    adjust=ON, 
    tieRotations=ON)



# Adjust mesh settings for CU and SI parts near the contact surface
# Seed the edges near the contact surface with the same size

# Assign coupled temperature-displacement element type to CU and SI parts
cu_region = (cu_part.cells, )
si_region = (si_part.cells, )

# Define the element type for coupled temperature-displacement analysis
cu_elem_type = ElemType(elemCode=C3D8T, elemLibrary=STANDARD)
si_elem_type = ElemType(elemCode=C3D8T, elemLibrary=STANDARD)

# Assign the element type to the regions
cu_part.setElementType(regions=cu_region, elemTypes=(cu_elem_type,))
si_part.setElementType(regions=si_region, elemTypes=(si_elem_type,))


contact_edges_cu = cu_part.edges.findAt(((mid_point[0], mid_point[1], 0),))
contact_edges_si = si_part.edges.findAt(((mid_point[0], mid_point[1], 0),))

cu_part.seedEdgeBySize(edges=contact_edges_cu, size=10.0, deviationFactor=0.1, constraint=FINER)
si_part.seedEdgeBySize(edges=contact_edges_si, size=10.0, deviationFactor=0.1, constraint=FINER)

# Generate the meshes
cu_part.generateMesh()
si_part.generateMesh()

# Create a job for the analysis
mdb.Job(name='ThermalAnalysis', model='Model-1', description='Thermal and mechanical analysis', 
         type=ANALYSIS, atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
         memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, explicitPrecision=DOUBLE, 
         nodalOutputPrecision=SINGLE, echoPrint=OFF, modelPrint=OFF, contactPrint=OFF, 
         historyPrint=OFF, userSubroutine='', scratch='', multiprocessingMode=DEFAULT, numCpus=1, numDomains=1, numGPUs=0)

mdb.saveAs(pathName='C:/Users/user/Desktop/ABAQUS/GEOMETRY/auto_model_output.cae')
# Submit the job and wait for completion
mdb.jobs['ThermalAnalysis'].submit(consistencyChecking=OFF)
mdb.jobs['ThermalAnalysis'].waitForCompletion()
