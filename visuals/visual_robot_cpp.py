import numpy as np
import matplotlib.pyplot as plt
import pandas as pds
import plotly.graph_objects as go

def vis_robot_interactive(pos_vec):
    '''
    
    '''

    fig = go.Figure()

    fig.add_trace(
        go.Scatter3d(
            x=pos_vec[:,0],
            y=pos_vec[:,1],
            z=pos_vec[:,2],
            name='Linkages',
            line=dict(
                color='black',
                width=10,
            )
        )
    )

    for i in range(len(pos_vec)):
        fig.add_trace(
            go.Scatter3d(
                x=[pos_vec[i,0]],
                y=[pos_vec[i,1]],
                z=[pos_vec[i,2]],
                name=f'Joint_{i}',
                marker=dict(
                    size=12,
                    color=np.sqrt(pos_vec[i,0]**2 + pos_vec[i,1]**2 + pos_vec[i,2]**2),
                    colorscale='Viridis',   # choose a colorscale
                    opacity=0.8
                )
            )
        )
    
    fig.update_layout(
        scene = dict(
            xaxis = dict(nticks=4, range=[-600,600],),
            yaxis = dict(nticks=4, range=[-600,600],),
            zaxis = dict(nticks=4, range=[-600,600],),
        )
    )

    fig.write_html('test_interactive.html')

cur_coords = np.genfromtxt('coords_joint.txt', dtype=float, 
                               delimiter=',', skip_header=1,
                               usecols=(1,2,3))

x = cur_coords.shape[0]
y = cur_coords.shape[1]

temp = np.zeros((x+1 ,y))
temp[1:] = cur_coords 
cur_coords = temp

# print(cur_coords)

vis_robot_interactive(cur_coords)

