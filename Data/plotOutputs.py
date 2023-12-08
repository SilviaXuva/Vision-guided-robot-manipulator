import pandas as pd
import glob
import numpy as np

def plotOutputs(number_joints = 7, measures = ['', '_dot', '_dot_dot'], cart = True, joint = True, folder = r'.\Outputs\Joint Controller\Joint Rtb Trajectory\2023-12-05_16h45m35s', show = False):
    if not show:
        import matplotlib
        matplotlib.use('Agg')
    else:
        import matplotlib
    import matplotlib.pyplot as plt

    options = {
        'Cart Comparison':{
            'data': 'x',
            'height': 5,
            'width': 15,
            'figures': [
                {
                    'data': [['x', 'y', 'z']],
                    'name': 'Trans'
                }, 
                {
                    'data': [['rx', 'ry', 'rz']],
                    'name': 'Rot'
                }
            ]
        },
        'Joint Comparison': {
            'data': 'q',
            'height': 30,
            'width': 15,
            'figures': [
                {
                    'data': [[f'q{i}'] for i in range(number_joints)],
                    'name': 'All joints'
                }
            ]
        }
    }
    if not cart:
        del options['Cart Comparison']
    if not joint:
        del options['Joint Comparison']

    for target_folder in glob.glob(f'{folder}\*\*'):
        for key in list(options.keys()):
            for measure in measures:
                for figure in options[key]['figures']:
                    fig, ax = plt.subplots()
                    fig.suptitle(f'{figure["name"]}: {options[key]["data"]}{measure}')
                    fig.set_figheight(options[key]['height'])
                    fig.set_figwidth(options[key]['width'])
                    rows, cols = np.array(figure['data']).shape
                    for i, df_column in enumerate(np.array(figure['data']).flatten()): 
                        for type, color in [['Real', 'red'], ['Ref', 'blue']]:
                            ax = plt.subplot(rows, cols, i + 1)
                            file = glob.glob(f'{target_folder}\*\{options[key]["data"]}{measure}_{type}.csv')[0]
                            df = pd.read_csv(file, index_col=0)
                            df = df.rename(columns={df_column: f'{df_column}{measure}_{type}'})
                            df[f'{df_column}{measure}_{type}'].plot(x=df.index, ax=ax, legend=True, color=color)
                            ax.title.set_text(f'{df_column}{measure}')
                    plt.show()
                    fig.savefig(fr'{target_folder}\{key}\{options[key]["data"]}{measure}_{figure["name"]}.png')
            