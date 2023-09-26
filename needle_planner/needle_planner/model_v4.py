from PyNite import FEModel3D
import numpy as np
from tqdm import tqdm
from matplotlib import pyplot as plt
import matplotlib.patches as patches
import sys

alp = 3.14/6 # needle tip angle
E = 35*10**3 # Elastic Modulus of Needle (Nitinol, MPa)
G = E/(2*(1+.33)) # Shear Modulus of Needle (Nitinol, MPa)
outer_diameter = 1.270 # mm
inner_diameter = 0.838 # mm
area = np.pi * (outer_diameter/2)**2 - np.pi * (inner_diameter/2)**2 # mm^2
J = np.pi/32*(outer_diameter**4 - inner_diameter**4) # Polar Moment of Inertia (tube cross section
I = np.pi/64*(outer_diameter**4 - inner_diameter**4) # Moment of Inertia (tube cross section)
guide_gap = 5 # mm
guide_thickness = 10 # mm

def run_model(tip_pos, y_rxn, y_disp, stiffness_vec, pos_vec, tip_path, guide_pos_vec, guide_pos_ins):
    def guide_pos(tip_pos):
        for i in range(len(guide_pos_ins)):
            if tip_pos < guide_pos_ins[i]:
                return guide_pos_vec[i]
        return guide_pos_vec[-1] 
        
    def tissue_stiffness(x):
        for i in range(len(pos_vec)):
            if x < pos_vec[i]:
                return stiffness_vec[i]
        return stiffness_vec[-1]
    boef = FEModel3D()
    j = 70
    node_spacing = 2 # mm
    # Assign the length of the beam
    L = 140  # mm
    offest = tip_pos - L

    num_nodes = round(L/3) + 1
    spacing = L/round(L/3)

    # Displacement array for the nodes in the tissue
    # disp_array = np.zeros(num_nodes*2)
    # disp_array[len(disp_array)-len(tip_path):] = np.array(tip_path)
    # Generate the nodes
    for i in range(num_nodes):
        node_x = i*spacing + offest
        # Add nodes spaced at 1mm
        boef.add_node('N' + str(i + 1), node_x, 0, 0)
        boef.add_node_load('N' + str(i + 1), 'FY', y_rxn[i-1])
        # boef.def_node_disp('N' + str(i + 1), 'DY', disp_array[i])
        # Fix nodes before the template, add spring supports

        if node_x < -guide_gap :
            # pass
            boef.def_node_disp('N' + str(i + 1), 'DY', guide_pos(tip_pos))
            # boef.def_support('N' + str(i + 1), True, True, True, True, True, True)
        elif node_x > 0 and not node_x == tip_pos:
            boef.def_support_spring('N' + str(i + 1), 'DY', tissue_stiffness(node_x), None)
        elif (not node_x == tip_pos) and (node_x < -guide_gap):
            boef.def_support('N' + str(i + 1), True, True, True, True, True, True)
    boef.def_support('N' + str(1), True, True, True, True, True, True)
    # Define member material properties
    boef.add_material('Nitinol', E, G, 0.33, 6.45e-9)

    # Define section properties (W8x35)
    A = area    # mm^2
    Iz = I    # mm^4 (strong axis)
    Iy = I   # mm^4 (weak axis)
    # J = J  # mm^4

    # Define the member
    boef.add_member('M1', 'N1', 'N' + str(num_nodes), 'Nitinol', Iy, Iz, J, A)

    # Add a point load at end
    end_node = 'N' + str(num_nodes)
    tissue_stiffness_tip = tissue_stiffness(tip_pos)
    force = tissue_stiffness_tip*outer_diameter**2/(4*np.tan(alp/2)**2)* np.cos(alp + np.arctan(y_disp[-2]-y_disp[-1]/spacing))
    boef.add_node_load(end_node, 'FY', -force)

    boef.analyze(check_statics=False, check_stability=True)

    # Find displacement and reaction in Y
    # print(boef.Nodes[end_node].RY['Combo 1'])
    y_disp = []
    y_rxn = []
    for i in range(num_nodes):
        y_disp.append(boef.Nodes['N'+str(i+1)].DY['Combo 1'])
        y_rxn.append(boef.Nodes['N'+str(i+1)].RxnFY['Combo 1'])
    tip_path.append(y_disp[-1])
    
    return np.array(y_disp), np.array(y_rxn), tip_path

# def calc_path(layers):
#     def inner_func(single_rand_vec):
#         stiffness = single_rand_vec[:layers]
#         pos_vec = single_rand_vec[layers:]
#         y_rxn = np.zeros(50)
#         y_disp = np.zeros(50)
#         name = 1
#         for tip_pos in range(1,100, 1):
#             y_disp, y_rxn = run_model(tip_pos, y_rxn, y_disp, stiffness, pos_vec)
#             # shift all elements by one place to the left
#             y_rxn = np.roll(y_rxn, -1)
#             # replace the last element with zero
#             y_rxn[-1] = 0
#         return y_disp[-1], y_disp, y_rxn, single_rand_vec
#     return inner_func

class CalcPath:
    def __init__(self, layers):
        self.layers = layers

    def __call__(self, single_rand_vec):
        stiffness = single_rand_vec[:self.layers]
        pos_vec = single_rand_vec[self.layers:2*self.layers-1]
        guide_pos_vec = single_rand_vec[2*self.layers-1:3*self.layers-1]
        guide_pos_ins = single_rand_vec[3*self.layers-1-1:]
        guide_pos_ins.sort()
        pos_vec.sort()
        y_rxn = np.zeros(141)
        y_disp = np.zeros(141)
        tip_path = []
        full_paths = []
        full_rxns = []
        for tip_pos in tqdm(range(1,125, 1)):
            y_disp, y_rxn, tip_path = run_model(tip_pos, y_rxn, y_disp, stiffness, pos_vec, tip_path, guide_pos_vec, guide_pos_ins)
            # shift all elements by one place to the left
            y_rxn = np.roll(y_rxn, -1)
            # replace the last element with zero
            y_rxn[-1] = 0
            
            # plt.close()
            full_paths.append(y_disp)
            full_rxns.append(y_rxn)
        # fig, ax = plt.subplots()
        # ax.scatter(np.arange(0,143,3)-40-100+tip_pos, y_disp)
        
        # ax.axvline(x=0, color='r', linestyle='dotted')
        # Create a Rectangle patch
        # rectangle = patches.Rectangle((-guide_gap-guide_thickness, 0), guide_thickness, guide_pos(tip_pos), linewidth=1, edgecolor='k', linestyle='dotted', facecolor='none')

        # Add the Rectangle patch to the axis
        # ax.add_patch(rectangle)    
        # ax.set_ylim(-15, 7)
        # ax.set_xlim(-150, 150)
        # plt.show()
        # plt.savefig(str(tip_pos) + '.png')
        return y_disp[-1], y_disp, y_rxn, single_rand_vec, np.array(full_paths), np.array(full_rxns)


def run_main(stiffness_vec, pos_vec, guide_pos_vec, guide_pos_ins):
    stiffness_vec = np.array(stiffness_vec).reshape(1,-1)
    pos_vec = np.array(pos_vec).reshape(1,-1)
    guide_pos_vec = np.array(guide_pos_vec).reshape(1,-1)
    guide_pos_ins = np.array(guide_pos_ins).reshape(1,-1)
    rand_vec = np.concatenate((stiffness_vec, pos_vec, guide_pos_vec, guide_pos_ins), axis=1)

    n_layers = 10
    y_tip, y_disp, y_rxn, single_rand_vec, full_paths, full_rxns = CalcPath(layers = n_layers)(rand_vec[0])
    return y_tip, y_disp, y_rxn, single_rand_vec, full_paths, full_rxns

if __name__ == '__main__':
    # file_name = str(sys.argv[1]) + '.npy'
    # from multiprocessing import Pool
    n = 1
    # 10 tissue layers and 10 guide positions
    n_layers = 10
    # stiffness_vec = np.random.uniform(.01, .08, (n,n_layers))  # Varying the cons_stiffness
    stiffness_vec = np.array([[0.07378714, 0.02571705, 0.06073464, 0.04500983, 0.0574037 ,
        0.07549429, 0.02845385, 0.0725467 , 0.05615388, 0.07357154]])
    # pos_vec = np.random.uniform(0, 120, (n,n_layers-1))
    pos_vec = np.array([[ 94.51560158,  47.03261747,  87.1245099 ,  89.46605554,
         17.40223147,  88.68403298,  49.51901644, 116.95763035,
         33.9627116 ]])
    # guide_pos_ins = np.random.uniform(0, 120, (n,n_layers-1))
    guide_pos_ins = np.array([[ 69.72243379,  96.14450312,  42.74242474,  87.55119006,
          8.00183706,  15.36813105, 104.24355278,   7.54419621,
         71.56735949]])
    # guide_pos_vec = np.random.uniform(-6, 6, (n,n_layers))
    guide_pos_vec = np.array([[ 5.97327943,  5.10077935, -2.27243571,  3.52361787, -2.46175497,
         3.9850011 , -5.76966477,  4.99527877, -0.52339698,  4.09840042]])
    guide_pos_vec[0,0]=0
    rand_vec = np.concatenate((stiffness_vec, pos_vec, guide_pos_vec, guide_pos_ins), axis=1)

    y_tip, y_disp, y_rxn, single_rand_vec, full_paths, full_rxns = CalcPath(layers = n_layers)(rand_vec[0])
    print(y_tip)
    # np.save('results/path_'+file_name, full_paths)
    # np.save('results/rxn_'+file_name, full_rxns)
    # np.save('results/in_'+file_name, single_rand_vec)

# np.save('paths5.npy', needle_paths)
# np.save('stiffness_pos5.npy', rand_vec)
