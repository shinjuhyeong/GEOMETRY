from abaqus import *
from abaqusConstants import *
import numpy as np
from scipy.interpolate import lagrange
#-------------------------------------------------
#Units are in MMKS(mm, kg, s, kg/m^3, Pa)
#Geometrical informations, Geometry is scaled up to 1000 times
R_SI = 100
Y_SI = 100
R0_CU = np.array([15, 35, 10])
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

cu_instance = myassembly.Instance(name='Cu', part=cu_part, dependent=ON)
si_instance = myassembly.Instance(name='Si', part=si_part, dependent=ON)

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

# 모델 저장
mdb.saveAs(pathName='C:/Users/user/Desktop/ABAQUS/GEOMETRY/auto_model_output.cae')
