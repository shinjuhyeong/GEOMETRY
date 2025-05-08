from abaqus import *
from abaqusConstants import *
import numpy as np
from scipy.interpolate import lagrange
#-------------------------------------------------
#Units are in MMKS(mm, N, ton, MPa, mJ, ton/mm^3)
#Geometrical informations
R_SI = 100
Y_SI = 100
R0_CU = np.array([15, 35, 10])
Y0_CU = np.array([0, 60, Y_SI])
#-------------------------------------------------
#Units are in MMKS(mm, N, ton, MPa, mJ, ton/mm^3)
#Material Properties(Cu)
E_CU = 120e3 # Elastic Modulus
NU_CU = 0.34 # Poisson's Ratio
CTE_CU = 17e-6
Y_CU = 150
def s_t_cu(e_t_cu): #Plasticity true stress-strain curve
    if e_t_cu <= 0.02:
        return 8.0e4 * e_t_cu
    else:
        return 69.6 * (e_t_cu ** 0.286)
P_TABLE_CU = ((s_t_cu(e_t_cu), e_t_cu) for e_t_cu in np.linspace(0, 0.2, 200, endpoint=True).tolist())
K_CU = 401e-3 #Thermal Conductivity
C_CU = 390e6 #Specific heat
RHO_CU = 8960e-12 #Density

#Material Properties(Si)
E_SI = 130e3
NU_SI = 0.28
CTE_SI = 2.8e-6
K_CU = 0.0015e-3
C_CU = 700e6
RHO_CU = 2330e-12

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


'''
#-------------------------------------------------
# 구리 재료 정의
cu_material = mymodel.Material(name = 'Cu')
cu_material.Elastic(table = (E_CU, NU_CU)) #E, Poisson's ratio
cu_material.Plastic(table = P_TABLE_CU)
cu_material

si_material = mymodel.Material(name = 'Si')
si_material.Elastic(table = (E_SI, NU_SI))
si_material

#-------------------------------------------------
'''
# 모델 저장
mdb.saveAs(pathName='C:/Users/user/Desktop/ABAQUS/GEOMETRY/auto_model_output.cae')
