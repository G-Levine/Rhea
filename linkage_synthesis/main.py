from symforce import symbolic as sf

"""
Forward kinematics for the four-bar linkage.
"""
def forward_kinematics(theta, parameters):
    # Define the symbolic variables.
    input_link_root = sf.V2.symbolic("input_link_root")
    passive_link_root = sf.V2.symbolic("passive_link_root")
    passive_link_tip = sf.V2.symbolic("passive_link_tip")
    
    input_link_length = sf.Symbol("input_link_length")
    passive_link_length = sf.Symbol("passive_link_length")
    output_link_length = sf.Symbol("output_link_length")

