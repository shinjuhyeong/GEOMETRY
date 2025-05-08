from abaqus import *
from abaqusConstants import *

# Function to extract and save model information
def save_model_info():
    mdb = openMdb(pathName='c:/Users/user/Desktop/ABAQUS/GEOMETRY/stress2.cae')
    model = mdb.models['Model-1']
    info_file_path = 'stress2info.txt'

    with open(info_file_path, 'w') as f:
        # Material Information
        f.write("Materials:\n")
        for material_name, material in model.materials.items():
            f.write(f"Material Name: {material_name}\n")
            for property_name, property_value in material.__dict__.items():
                f.write(f"  {property_name}: {property_value}\n")
            f.write("\n")

        # Section Information
        f.write("Sections:\n")
        for section_name, section in model.sections.items():
            f.write(f"Section Name: {section_name}\n")
            for property_name, property_value in section.__dict__.items():
                f.write(f"  {property_name}: {property_value}\n")
            f.write("\n")

        # Boundary Condition Information
        f.write("Boundary Conditions:\n")
        for bc_name, bc in model.boundaryConditions.items():
            f.write(f"BC Name: {bc_name}\n")
            for property_name, property_value in bc.__dict__.items():
                f.write(f"  {property_name}: {property_value}\n")
            f.write("\n")

        # Load Information
        f.write("Loads:\n")
        for load_name, load in model.loads.items():
            f.write(f"Load Name: {load_name}\n")
            for property_name, property_value in load.__dict__.items():
                f.write(f"  {property_name}: {property_value}\n")
            f.write("\n")

    print(f"Model information saved to {info_file_path}")

# Call the function to save model information
save_model_info()