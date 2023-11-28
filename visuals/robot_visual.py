import numpy as np
import matplotlib.pyplot as plt
import pandas as pds
import plotly.graph_objects as go

def make_link_param_df(*links):
    '''
    
    '''
    df_dict = {}

    count = 1
    for i in links:
        df_dict[f'Link_{count}'] = i
        count += 1

    link_param = pds.DataFrame(df_dict, index=['a_i','al_i','d_i','th_i'])

    print(link_param)    

def make_A_i(a, al, d, th):
    '''
    inputs:
        a : link length
        al : link twist
        d : link offset
        th : joint angle
    '''

    A_i = np.array([
        [np.cos(th), -np.sin(th)*np.cos(al), np.sin(th)*np.sin(al), a*np.cos(th)],
        [np.sin(th), np.cos(th)*np.cos(al), -np.cos(th)*np.sin(al), a*np.sin(th)],
        [0, np.sin(al), np.cos(al), d],
        [0, 0, 0, 1]
    ], dtype='f8')

    return A_i

def PEM_setup(*links):
    '''
    inputs:
        *links : link parameters; np.array([a_i, al_i, d_i, th_i])
    '''

    df_dict = {}

    count = 1
    for i in links:
        df_dict[f'Link_{count}'] = i
        count += 1

    A_dict = {}
    T_dict = {}

    count = 1
    for i in df_dict:
        A_dict[f'A_{count}'] = make_A_i(*df_dict[i])

        for j in range(len(A_dict)):
            if j == 0:
                temp_T = A_dict[f'A_{j+1}']
            else:
                temp_T = np.dot(temp_T,A_dict[f'A_{j+1}'])

        T_dict[f'T^0_{count}'] = temp_T

        count += 1

    return T_dict

def vis_robot(T_dict):
    '''

    '''

    fig = plt.figure(figsize=(5,5))
    ax1 = plt.axes(projection='3d')

    pos_vec = np.zeros((len(T_dict)+1,3))

    index = 1
    for i in T_dict:

        pos_vec[index] = T_dict[i][:-1,3]

        index+=1
        
    ax1.plot3D(pos_vec[:,0], pos_vec[:,1], pos_vec[:,2])

    for i in range(len(pos_vec)):
        ax1.scatter3D(pos_vec[i,0], pos_vec[i,1], pos_vec[i,2], label=f'Joint {i}')

    ax1.legend()

    plt.savefig('test.pdf')
    plt.close()

def vis_robot_interactive(T_dict):
    '''
    
    '''
    pos_vec = np.zeros((len(T_dict)+1,3))

    index = 1
    for i in T_dict:

        pos_vec[index] = T_dict[i][:-1,3]

        index+=1

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

    fig.write_html('test_interacti')

A = 1276
B = 1620
G = 420
H = 455
I = 400
J = 855

th_1 = 0
th_2 = 0
th_3 = 0
th_4 = 0
th_5 = np.pi/2
th_6 = 0

links = np.array([
    [25, -np.pi/2, I, th_1],
    [H, 0, 0, th_2],
    [35, np.pi/2, 0, th_3],
    [0, -np.pi/2, -G, th_4],
    [0, np.pi/2, 0, th_5],
    [0, np.pi, -80, th_6]
])

# make_link_param_df(*links)

T_dict = PEM_setup(*links)

vis_robot(T_dict)
vis_robot_interactive(T_dict)