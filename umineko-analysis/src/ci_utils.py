import os
import glob
import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as tck
import seaborn as sns
from IPython.display import display
import src.utils as utils
from src.utils import get_speaker_turn_on_idx

palette0 = sns.color_palette(['#E69F00', '#56B4E9', '#009E73', '#F0E442', '#0072B2', '#D55E00', '#CC79A7', '#000000'])
palette2 = sns.color_palette(["#D81B60", "#1E88E5", "#FFC107", "#004D40"])


"""

Causal Inference analysis
- Data preparation for analysis using R on post- and pre-mean values
- Data preparation for CausalImpact 
- Data visualization

"""

# load the config file
def load_from_yaml(filename):
    with open(filename, 'r') as file:
        config = yaml.safe_load(file)
    return config


def setup_for_causal_impact_analysis(
    data_type,
    use_covariates=False
):
    # Acceleration
    if data_type == "smoothed_VeDBA_2s":
        # title_base = "Smoothed VeDBA"
        title_base = "S-VeDBA"
        data_dirname = "extracted-imu-data/*"
    # GPS
    elif data_type == "speed_distance_km_h":
        title_base = "Speed"
        data_dirname = "extracted-gps-data/*"
    elif data_type == "abs_diff_distance_m": # m/s (1 Hz)
        # title_base = "Abs. Diff. Speed"
        title_base = "AD-Speed"
        data_dirname = "extracted-gps-data/*"
    # Video
    elif data_type == "pixel_count_p": # umineko-2024-v8i-yolov8
        title_base = "MPR"
        # title_base = "Masked Pixel Ratio"
        data_dirname = "umineko-2024-v8i-yolov8"
    elif data_type == "abs_diff_pixel_count_p":
        # title_base = "Abs. Diff. MPR"
        title_base = "AD-MPR"
        data_dirname = "umineko-2024-v8i-yolov8"
    else:
        raise ValueError(f"{data_type} is not appropriate.")
    
    if use_covariates == True:
        # If you want to include covariates in the model, 
        # add the covariates here that satisfy the parallel trend assumption
        X_cols = ['speaker_on', 'X1', 'X2', 'X3']
    else:
        # No covariates
        X_cols = [] 

    Y_col = [f"{data_type}"]
    target_path = f"../data/{data_dirname}/*.csv"
    path_list = sorted(glob.glob(target_path))

    return path_list, Y_col, X_cols, title_base


def prep_df_pre_post_analysis(
    data_type="abs_diff_distance_m",
    duration_sec=5,
    value_type='mean'
):
    
    # load metadata
    df_meta = pd.read_csv('../data/metadata/session_data.csv')
    _df_meta = df_meta[df_meta['file_name'] != 'LBP01_S03']
    # _df_meta = df_meta[df_meta['audio_file_name'] != 'Cancelled']
    # print(len(df_meta))
    # print(len(_df_meta))
    audio_file_name_list = list(_df_meta['audio_file_name'].values)
    pb_count_list = list(_df_meta['pb_count'].values)
    pb_count_2_list = list(_df_meta['pb_count_2'].values)

    unique_test_id_list = np.unique(df_meta['test_id'])
    # print(unique_test_id_list)

    # fetch path for sensor data and target column name
    (
        path_list, Y_col, _, title_base
    ) = setup_for_causal_impact_analysis(data_type, use_covariates=False)
    # print(len(path_list))
    target = Y_col[0]

    test_id_list = []
    session_name_list = []
    pre_value_list = []
    post_value_list = []
    diff_value_list = []

    
    for i, path in enumerate(path_list):
        session_name = os.path.basename(path).replace(".csv", "")
        _session_name = os.path.basename(path).replace(".csv", "").replace("_", ":")
        # print(f"session_name: {session_name}")
        df = pd.read_csv(path, low_memory=False)
        # print(df.columns)
        # audio_file, audio_file_name = utils.identify_audio_file(df)
        pre_value, post_value = calc_pre_post_data_per_session(
            df,
            duration_sec=duration_sec,
            target=target,
            value_type=value_type
        )
        test_id_list.append(session_name[:5])
        session_name_list.append(session_name)
        pre_value_list.append(pre_value)
        post_value_list.append(post_value)
        diff_value_list.append(post_value - pre_value)

    # print(len(test_id_list))
    data_dict = {
        'test_id': test_id_list,
        'session_id': session_name_list,
        'pre_value': pre_value_list,
        'post_value': post_value_list,
        'diff_value': diff_value_list,
        'audio_file_name': audio_file_name_list,
        'pb_count': pb_count_list,
        'pb_count_2': pb_count_2_list,
    }
    df = pd.DataFrame(data_dict)

    data_dict_2 = {
        'test_id': test_id_list *2,
        'session_id': session_name_list *2,
        'pre_post': ['Pre'] * len(test_id_list) + ['Post'] * len(test_id_list),
        'value': pre_value_list + post_value_list,
        'audio_file_name': audio_file_name_list *2,
    }
    df2 = pd.DataFrame(data_dict_2)
    # print(len(df))
    # print(len(df2))
    y_val = df2['value']
    y_diff_val = df['diff_value']
    s1 = f'target column: {target}'
    s2 = f'min = {np.min(y_val):.2f}, max = {np.max(y_val):.2f}'
    s3 = f'diff_min = {np.min(y_diff_val):.2f}, diff_max = {np.max(y_diff_val):.2f}'
    print(f'{s1} | {s2} | {s3}')
    return df, df2

def calc_pre_post_data_per_session(
    df, 
    duration_sec=5, 
    target='abs_diff_distance_m',
    value_type='mean'
):

    # fetch audio playback start index
    idx = get_speaker_turn_on_idx(df)
    if idx == 0:
        return 0, 0
        
    if target in ['speed_distance_m_s', 'speed_distance_km_h', 'abs_diff_distance_m']:
        idx_shift = duration_sec
    elif target in ['smoothed_VeDBA_2s']:
        idx_shift = duration_sec * 25 # 25 Hz
    elif target in ['pixel_count_p', 'abs_diff_pixel_count_p']:
        idx_shift = duration_sec * 30 # 30 FPS
    else:
        raise Exception(f'target: {target}')
    
    df_pre = df[ idx - idx_shift : idx ]
    df_post = df[ idx : idx + idx_shift ]
    pre_data = df_pre[target].values
    post_data = df_post[target].values

    # calc mean values pre- and post-periods data
    if value_type == 'mean':
        pre_value = np.mean(pre_data)
        post_value = np.mean(post_data)
    elif value_type == 'median':
        pre_value = np.median(pre_data)
        post_value = np.median(post_data)

    return pre_value, post_value


# Exception handler
def input_all_zeros_handler(data, pre_period):
    np.random.seed(558)
    random_pre_data_index = np.random.randint(pre_period[0], pre_period[1]+1)
    very_small_value = 0.0000001 # 1e-7
    data.loc[random_pre_data_index, 'y'] += very_small_value
    return data

def vis_slope_plot_ax(df, df2, ax):
    
    palette0 = sns.color_palette(['#E69F00', '#56B4E9', '#009E73', '#F0E442', '#0072B2', '#D55E00', '#CC79A7', '#000000']) # Okabe-Ito
    palette1 = sns.color_palette(["#ff4554", "#00bbdf", "#bad600", "#f02d7d", "#f8b62e", "#8b26a6","#808080"]) # https://anoiro.com/themes/switch-joycons
    palette2 = sns.color_palette(["#D81B60", "#1E88E5", "#FFC107", "#004D40"])
    # palette = palette0
    # palette = palette1
    palette = palette2
    N = len(df)
    _df2 = df2[df2['audio_file_name'] != 'Cancelled']
    max_value = np.max(_df2['value'])
    min_value = np.min(_df2['value'])
    pre_value_list = df['pre_value']
    post_value_list = df['post_value']
    diff_value_list = df['diff_value']
    audio_file_name_list = df['audio_file_name']
    for i in range(N):
        if diff_value_list[i] > 0:
            COLOR = palette[0]
            LABEL = "A"
        else:
            COLOR = palette[1]
            LABEL = "B"
        
        if audio_file_name_list[i] != 'Cancelled':
            ax.plot(
                [0.2, 0.8], 
                [pre_value_list[i], post_value_list[i]], 
                marker='o', markersize=10, color=COLOR, label=LABEL, alpha=0.5,
            )

    sns.boxplot(ax=ax, data=_df2, x='pre_post', y='value', showfliers=False, width=0.2, linewidth=2.0)
    # sns.violinplot(ax=ax, data=_df2, x='pre_post', y='value', width=0.1, color="white")
    for patch in ax.patches:
        r, g, b, a = patch.get_facecolor()
        patch.set_facecolor((r, g, b, .0))
    ax.set_xlim([-0.35, 1.35])
    # ax.set_ylim([min_value - 0.3*np.abs(min_value), max_value+0.1*np.abs(max_value)])
    # ax.set_xlabel(None, labelpad=10)
    ax.grid(which='major', axis='y')
    ax.set_xlabel("", labelpad=10)
    ax.set_ylabel("Value", labelpad=10)
    ax.yaxis.set_major_formatter(tck.FormatStrFormatter('%.2f'))
    ax.legend().remove()
    # plt.show()
    return ax


def vis_line_plot_pre_post_diff_ax(ax, df, data_type):
    unique_test_id_list = np.unique(df['test_id'])
    min_value = np.min(df['diff_value'])
    max_value = np.max(df['diff_value'])
    scatter = sns.scatterplot(
        x='pb_count', y='diff_value', style='audio_file_name', hue='test_id', hue_order=unique_test_id_list, data=df, ax=ax
    )
    lines = sns.lineplot(x='pb_count', y='diff_value', hue='test_id', hue_order=unique_test_id_list, data=df, ax=ax, legend=False)
    handles, labels = scatter.get_legend_handles_labels()
    # ax.legend(handles, labels, ncol=2)
    ax.legend().remove()
    ax.grid(which='major')
    title = f"{data_type}"
    ax.set_title(title, pad=10)
    # ax.set_xlabel('Playback count', labelpad=10)
    ax.set_ylabel('Diff. value', labelpad=10)
    ax.set_xticks(np.arange(0, 12, 1))
    ax.set_xlim(0.5, 11.5)
    ax.set_ylim(min_value-np.abs(min_value)*0.5, max_value+np.abs(max_value)*0.5)
    ax.yaxis.set_major_formatter(tck.FormatStrFormatter('%.2f'))
    ax.legend().remove()
    
    # plt.show()
    return ax, handles, labels


def vis_line_plot_pre_post_diff_2_ax(ax, df, data_type, title, audio_file):
    # display(df)
    
    unique_test_id_list = np.unique(df['test_id'])
    max_value = np.max(df['diff_value'])
    min_value = np.min(df['diff_value'])
    _df = df[df['audio_file_name'] == audio_file] 
    title = f'{audio_file} | {title}'
    
    scatter = sns.scatterplot(x='pb_count_2', y='diff_value', hue='test_id', hue_order=unique_test_id_list, data=_df, ax=ax)
    lines = sns.lineplot(x='pb_count_2', y='diff_value', hue='test_id', hue_order=unique_test_id_list, data=_df, ax=ax, legend=False)
    ax.legend().remove()

    ax.set_title(title, pad=10)
    ax.set_xlabel('Playback count', labelpad=10)
    ax.set_ylabel('Diff. value', labelpad=10)
    ax.grid(which='major', axis='x', alpha=0.7)
    ax.grid(which='major', axis='y', alpha=0.7)
    ax.grid(which='minor', axis='y', alpha=0.3)
    ax.set_xticks(np.arange(0, 12, 1))
    ax.set_xlim(0.5, 9.5)
    
    # ax.set_ylim(min_value-0.55*np.abs(min_value), max_value+0.25*np.abs(max_value))
    if data_type == 'smoothed_VeDBA_2s':
        ax.set_yticks(np.arange(-2.0, 2.0, 0.5))
        ax.set_ylim(-0.59, 0.59)

    elif data_type == 'speed_distance_km_h':
        ax.set_yticks(np.arange(-40, 100, 20))
        ax.set_ylim(-25, 55)
    if data_type == 'abs_diff_distance_m':
        ax.set_yticks(np.arange(-4.0, 4.0, 2.0))
        ax.set_ylim(-2.5, 2.5)
    
    elif data_type == 'pixel_count_p':
        ax.set_yticks(np.arange(-0.20, 0.5, 0.1))
        ax.set_ylim(-0.18, 0.13)
    elif data_type == 'abs_diff_pixel_count_p':
        ax.set_yticks(np.arange(-0.20, 0.5, 0.02))
        # ax.set_ylim(-0.025, 0.045)
        ax.set_ylim(-0.035, 0.045)
    ax.yaxis.set_minor_locator(tck.AutoMinorLocator(2))
    ax.yaxis.set_major_formatter(tck.FuncFormatter(utils.custom_formatter_2f))
    # plt.show()
    return ax, scatter

def prep_data_for_causal_impact_analysis(
    path, 
    Y_col, 
    X_cols,
    data_type='abs_diff_distance_m',
    ci_cfg=None
):
    # test_id = os.path.basename(path).replace(".csv", "")
    df = pd.read_csv(path)
    _df = df
    # display(df.head(3))
    # print(_df.columns)
    
    program_index = np.arange(1, len(df)+1, 1)
    _cols = ['_program_index'] + Y_col + X_cols
    
    idx = get_speaker_turn_on_idx(df)
    after_sec_offset = 1 # add 1 sec for prediction

    if ci_cfg is None:
        if data_type in ['speed_distance_km_h', 'abs_diff_distance_m']:
            sr = 1
            before_sec = 60
            after_sec = 25
        elif data_type in ['smoothed_VeDBA_2s']:
            sr = 25
            before_sec = 30
            after_sec = 25
        elif data_type in ['pixel_count_p', 'abs_diff_pixel_count_p']:
            sr = 30
            before_sec = 30
            after_sec = 25
        else:
            raise ValueError(f"{data_type} is not appropriate.")
    else: # use config file information
        sr = ci_cfg['data'][data_type]['sr'] 
        before_sec = ci_cfg['data'][data_type]['before_sec'] 
        after_sec = ci_cfg['data'][data_type]['after_sec'] 
        # print(sr)
        # print(before_sec)
        # print(after_sec)
    
    if sr == 1:
        after_sec = after_sec + after_sec_offset

    start_idx = idx - before_sec*sr
    end_idx = idx + after_sec*sr

    if idx == 0:
        df_X, df_Y = None, None
    else:
        df = df[_cols].iloc[start_idx:end_idx]
        # print(f"idx: {idx}, len(df): {len(df)}")
        # df_Y = df[Y_col].iloc[idx-60:idx+65]
        # df_Y = df_Y.values
        # df_X = df[X_cols].iloc[idx-60:idx+65]
        # df_X = df_X.values
    
    # display(df)
    # print(df[df['_program_index'] == 0].index[0])
    # print(len(df))

    # Note that the data frame has idx-60:idx+65 data now.
    # Playback index will be shifted after running df.reset_index(inplace=True).
    df.reset_index(inplace=True)
    program_index_zeros = df[df['_program_index'] == 0].index
    if len(program_index_zeros) >= 1:
        intervention_start_index = program_index_zeros[0]
        # print(f"intervention_start_index: {intervention_start_index}")
    else:
        return None, None, None, None, 0
    pre_period = [0, int(intervention_start_index - 1)]
    post_period = [int(intervention_start_index), int(len(df)-1)]
    # print(pre_period)
    # print(post_period)

    data_cols = Y_col + X_cols
    data = df[data_cols]
    df = df.rename(columns={f'{Y_col[0]}': 'y'})
    data = data.rename(columns={f'{Y_col[0]}': 'y'})
    # display(data)

    return _df, data, pre_period, post_period, intervention_start_index

def save_ci_summary_report_as_txt(ci, filepath):
    with open(filepath, 'w', encoding='utf-8') as file:
        file.write("> print(ci.summary())\n")
        file.write(ci.summary())
        file.write("\n")
        file.write("\n")
        file.write("> print(ci.summary('report'))\n")
        file.write(ci.summary('report'))


def get_vis_ci_params(data_type, ci_cfg=None):
    sr = ci_cfg['data'][data_type]['sr']
    before_sec = ci_cfg['data'][data_type]['before_sec']
    after_sec = ci_cfg['data'][data_type]['after_sec']
    # print(type(after_sec))
    # print(sr)
    if after_sec >= 20:
        interval_sec = 10 if sr == 1 else 5
        xticks = np.arange(0, (before_sec+after_sec + 1)*sr, interval_sec*sr)
        xtick_labels = np.arange(-before_sec, after_sec + 1, interval_sec)
        xlim = ( (before_sec-after_sec + 1)*sr, (before_sec+after_sec - 1)*sr)
    elif after_sec == 15:
        interval_sec = 10 if sr == 1 else 5
        xticks = np.arange(0, (before_sec+after_sec + 1)*sr, interval_sec*sr)
        xtick_labels = np.arange(-before_sec, after_sec + 1, interval_sec)
        xlim = ( (before_sec-after_sec + 1)*sr, (before_sec+after_sec - 1)*sr)
    elif after_sec == 10:
        interval_sec = 10 if sr == 1 else 5
        xticks = np.arange(0, (before_sec+after_sec + 1)*sr, interval_sec*sr)
        xtick_labels = np.arange(-before_sec, after_sec + 1, interval_sec)
        xlim = ( (before_sec-after_sec + 1)*sr, (before_sec+after_sec - 1)*sr)
    elif after_sec == 5:
        interval_sec = 10 if sr == 1 else 5
        xticks = np.arange(0, (before_sec+after_sec + 1)*sr, interval_sec*sr)
        xtick_labels = np.arange(-before_sec, after_sec + 1, interval_sec)
        xlim = ( (before_sec-after_sec)*sr, (before_sec+after_sec - 1)*sr)

    if data_type in ['smoothed_VeDBA_2s']:
        # Blue
        PRED_LINE_COLOR = palette2[1]
        PRED_CI_COLOR = palette2[1]
        PRED_CI_ALPHA = 0.3
    
    elif data_type in ['speed_distance_km_h', 'abs_diff_distance_m']:
        # Yellow
        PRED_LINE_COLOR = palette2[2]
        PRED_CI_COLOR = palette2[2]
        PRED_CI_ALPHA = 0.3
    
    elif data_type in ['pixel_count_p', 'abs_diff_pixel_count_p']:
        # Red
        PRED_LINE_COLOR = palette2[0]
        PRED_CI_COLOR = palette2[0]
        PRED_CI_ALPHA = 0.3
    
    # xticks = np.arange(0, (before_sec+after_sec + 1)*sr, interval_sec*sr)
    # xtick_labels = np.arange(-before_sec, after_sec + 1, interval_sec)
    # xlim = ( (before_sec-after_sec)*sr - 1, (before_sec+after_sec)*sr + 1 )
    
    return xticks, xtick_labels, xlim, PRED_LINE_COLOR, PRED_CI_COLOR, PRED_CI_ALPHA


def plot_causal_impact_3_rows(
    a, ax, t, df, CI,
    xticks, xtick_labels, xlim,
    DATA_LINE_COLOR, PRED_LINE_COLOR, PRED_CI_COLOR, PRED_CI_ALPHA
):
    if a == 0:
        ax = sns.lineplot(
            ax=ax, x=t, y=df['y'], 
            color=DATA_LINE_COLOR, 
            label='Data'
        )
        ax.fill_between(
            x=t, y1=df['complete_preds_lower'], y2=df['complete_preds_upper'], 
            color=PRED_CI_COLOR, alpha=PRED_CI_ALPHA, label=f'{CI}% CI'
        )
        ax = sns.lineplot(
            ax=ax, x=t, y=df['complete_preds_means'], 
            color=PRED_LINE_COLOR, linestyle='--', label='Predicted'
        )
    elif a == 1:
        ax.axhline(y=0, xmin=-500, xmax=500, color="#808080", linestyle='-')
        ax.fill_between(
            x=t, y1=df['point_effects_lower'], y2=df['point_effects_upper'], 
            color=PRED_CI_COLOR, alpha=PRED_CI_ALPHA, label=f'{CI}% CI'
        )
        ax = sns.lineplot(
            ax=ax, x=t, y=df['point_effects_means'], 
            color=PRED_LINE_COLOR, linestyle='--', label='Point Effects'
        )
    else:
        ax.axhline(y=0, xmin=-500, xmax=500, color="#808080", linestyle='-')
        ax.fill_between(
            x=t, y1=df['post_cum_effects_lower'], y2=df['post_cum_effects_upper'], 
            color=PRED_CI_COLOR, alpha=PRED_CI_ALPHA, label=f'{CI}% CI'
        )
        ax = sns.lineplot(
            ax=ax, x=t, y=df['post_cum_effects_means'], 
            color=PRED_LINE_COLOR, linestyle='--', label='Cumulative Effects'
        )

    ax.set_xticks(xticks)
    ax.set_xticklabels(xtick_labels)
    ax.set_xlim(xlim)
    ax.xaxis.set_minor_locator(tck.AutoMinorLocator(2))
    ax.yaxis.set_minor_locator(tck.AutoMinorLocator(2))
    ax.yaxis.set_major_formatter(tck.FormatStrFormatter('%.1f'))
    ax.grid(which='major')
    ax.grid(which='minor')

    ax.set_ylabel("")
    if a == 2:
        ax.set_xlabel("Time (s)", labelpad=15)
    
    return ax


def plot_causal_impact_3_rows_r_version(
    a, ax, t, df, CI,
    xticks, xtick_labels, xlim,
    DATA_LINE_COLOR, PRED_LINE_COLOR, PRED_CI_COLOR, PRED_CI_ALPHA
):
    if a == 0:
        ax = sns.lineplot(
            ax=ax, x=t, y=df['response'], 
            color=DATA_LINE_COLOR, 
            label='Data'
        )
        ax.fill_between(
            x=t, y1=df['point.pred.lower'], y2=df['point.pred.upper'], 
            color=PRED_CI_COLOR, alpha=PRED_CI_ALPHA, label=f'{CI}% CI'
        )
        ax = sns.lineplot(
            ax=ax, x=t, y=df['point.pred'], 
            color=PRED_LINE_COLOR, linestyle='--', label='Predicted'
        )
        ax.set_ylabel("Original")
    elif a == 1:
        ax.axhline(y=0, xmin=-500, xmax=500, color="#808080", linestyle='-')
        ax.fill_between(
            x=t, y1=df['point.effect.lower'], y2=df['point.effect.upper'], 
            color=PRED_CI_COLOR, alpha=PRED_CI_ALPHA, label=f'{CI}% CI'
        )
        ax = sns.lineplot(
            ax=ax, x=t, y=df['point.effect'], 
            color=PRED_LINE_COLOR, linestyle='--', label='Point Effects'
        )
        ax.set_ylabel("Pointwise")
    else:
        ax.axhline(y=0, xmin=-500, xmax=500, color="#808080", linestyle='-')
        ax.fill_between(
            x=t, y1=df['cum.effect.lower'], y2=df['cum.effect.upper'], 
            color=PRED_CI_COLOR, alpha=PRED_CI_ALPHA, label=f'{CI}% CI'
        )
        ax = sns.lineplot(
            ax=ax, x=t, y=df['cum.effect'], 
            color=PRED_LINE_COLOR, linestyle='--', label='Cumulative Effects'
        )
        ax.set_ylabel("Cumulative")
    
    ax.yaxis.set_label_coords(-0.18, 0.5)

    ax.set_xticks(xticks)
    ax.set_xticklabels(xtick_labels)
    # ax.set_xlim(xlim)
    ax.xaxis.set_minor_locator(tck.AutoMinorLocator(2))
    ax.yaxis.set_minor_locator(tck.AutoMinorLocator(2))
    ax.yaxis.set_major_formatter(tck.FormatStrFormatter('%.1f'))
    ax.grid(which='major')
    ax.grid(which='minor')

    if a == 2:
        ax.set_xlabel("Time (s)", labelpad=15)
    
    return ax


def vis_causal_impact_axes(
    ax_list,
    df,
    alpha=0.03, 
    title="LBP10_S00",
    data_type='abs_diff_distance_m',
    ci_cfg=None,
    use_r_output=False,
    intervention_start_index=None,
    context="poster",
):
    """
    Args
    ----
        data: Union[np.array, pd.DataFrame]

        ci: 

        credible_intervals
    Returns
    -------
    """

    CI = int((1 - alpha) * 100)
    t = np.arange(0, len(df), 1)
    if intervention_start_index is None and use_r_output == False:
        intervention_start_index = df['post_preds_means'].first_valid_index()
    # print(f"intervention_start_index: {intervention_start_index}")
    
    (
        xticks, xtick_labels, xlim, PRED_LINE_COLOR, PRED_CI_COLOR, PRED_CI_ALPHA
    ) = get_vis_ci_params(data_type, ci_cfg)

    DATA_LINE_COLOR = palette0[-1] # black
    
    for a, ax in enumerate(ax_list):
        # print(a)
        ax.axvline(
            x=intervention_start_index, ymin=-500, ymax=500, 
            color="#555555", linestyle='--', linewidth=3.0
        )
        if use_r_output:
            ax = plot_causal_impact_3_rows_r_version(
                a, ax, t, df, CI,
                xticks, xtick_labels, xlim,
                DATA_LINE_COLOR, PRED_LINE_COLOR, PRED_CI_COLOR, PRED_CI_ALPHA
            )
        else:
            ax = plot_causal_impact_3_rows(
                a, ax, t, df, CI,
                xticks, xtick_labels, xlim,
                DATA_LINE_COLOR, PRED_LINE_COLOR, PRED_CI_COLOR, PRED_CI_ALPHA
            )
        
        handles, labels = ax.get_legend_handles_labels()
        if len(handles) == 2:
            handles = [handles[1], handles[0]]
            labels = [labels[1], labels[0]]
        elif len(handles) == 3:
            handles = [handles[0], handles[2], handles[1]]
            labels = [labels[0], labels[2], labels[1]]
        
        # for figure 04
        if "LBP01_S00" in title:
            legend_ncol = len(handles)
            legend_loc = 'upper left'
            if a == 2 and "Speed" in title:
                legend_loc = 'lower left'
            if "MPR" in title:
                legend_ncol = 1
        else:
            legend_ncol = 1
            legend_loc = 'upper left'
        
        adjust_factor = 1.8
        y_min, y_max = ax.get_ylim()
        if y_min > 0:
            ax.set_ylim(0 - (y_max-y_min)*0.1, y_max * adjust_factor)
        else:
            ax.set_ylim(y_min, y_max * adjust_factor)

        if context == "poster":
            ax.legend(handles, labels, ncol=legend_ncol, loc=legend_loc, fontsize=16)
            if a == 0:
                ax.set_title(f'{title.replace("_", " ")}', fontsize=24, fontweight='bold', pad=20)
        else:
            legend_ncol = 3
            legend_loc = 'upper left'
            ax.legend(handles, labels, ncol=legend_ncol, loc=legend_loc, fontsize=8)
            ax.set_title(f'{title.replace("_", " ")}', fontweight='bold', pad=10)
    # plt.tight_layout()
    # plt.show()
    
    return ax_list


def plot_causal_impact_per_session(
    session_id, 
    data_type_list, 
    ci_cfg
):
    run_id = ci_cfg['run_id']
    use_covariates = ci_cfg['ci']['use_covariates']

    GRIDSPEC_KW = {
        'wspace': 0.4, 
        'hspace': 0.6, 
        'width_ratios': [1.0, 1.0, 1.0], 
        'height_ratios': [1.0, 1.0, 1.0]
    }
    fig, axes = plt.subplots(3, 3, figsize=(13, 7), gridspec_kw=GRIDSPEC_KW)
    ax_idx = 0
    label_idx = 0
    plot_label_list = ['(a)', '(b)', '(c)']
    for j, data_type in enumerate(data_type_list):
        (
            path_list, Y_col, X_cols, title_base
        ) = setup_for_causal_impact_analysis(data_type, use_covariates)
        for i, path in enumerate(path_list):   
            if session_id in path:
                ax_row_idx = 0
            else:
                continue # skip

            session_name = os.path.basename(path).replace(".csv", "")
            
            (
                df, data, pre_period, post_period, intervention_start_index
            ) = prep_data_for_causal_impact_analysis(path, Y_col, X_cols, data_type, ci_cfg)
            
            # df_ci_path = f"../output/causal-impact/{run_id}/{data_type}/{session_name}/df_ci.csv"
            df_ci_path = f"../output/causal-impact/{run_id}/{data_type}/{session_name}/r/df_ci_r.csv"
            
            df_ci = pd.read_csv(df_ci_path)
            _data_type = data_type.replace("-", "_")
            file_name = f"fig_ci_{session_name.lower()}_{_data_type}"
            fig_title = f"{session_name} | {title_base}"
            ax_list = [axes[ax_row_idx, j], axes[ax_row_idx+1, j], axes[ax_row_idx+2, j]]
            ax_list = vis_causal_impact_axes(
                ax_list, 
                df_ci, 
                alpha=0.03, 
                title=fig_title, 
                data_type=data_type, 
                ci_cfg=ci_cfg,
                use_r_output=True,
                intervention_start_index=intervention_start_index,
                context="paper"
            )
            
            ax = axes[ax_row_idx, j]
            ax.text(
                s=f'{plot_label_list[label_idx]}', x=-0.27, y=1.3, 
                transform=ax.transAxes, fontsize=18, fontweight='bold'
            )
            ax_idx = ax_idx + 1
            label_idx = label_idx + 1

    # to add white space
    # for ax in axes[3]:
    #     ax.axis('off')

    return fig

def plot_figure_04(data_type_list, ci_cfg):
    run_id = ci_cfg['run_id']
    use_covariates = ci_cfg['ci']['use_covariates']

    GRIDSPEC_KW = {
        'wspace': 0.4, 
        'hspace': 0.4, 
        'width_ratios': [1.0, 1.0, 1.0], 
        'height_ratios': [1.0, 1.0, 1.0, 0.5, 1.0, 1.0, 1.0]
    }
    fig, axes = plt.subplots(7, 3, figsize=(26, 24), gridspec_kw=GRIDSPEC_KW)
    ax_idx = 0
    label_idx = 0
    plot_label_list = ['(a)', '(d)', '(b)', '(e)', '(c)', '(f)']
    for j, data_type in enumerate(data_type_list):
        (
            path_list, Y_col, X_cols, title_base
        ) = setup_for_causal_impact_analysis(data_type, use_covariates)
        for i, path in enumerate(path_list):   
            if "LBP01_S00" in path:
                ax_row_idx = 0
            elif "LBP03_S00" in path:
                ax_row_idx = 4
            else:
                continue # skip
            session_name = os.path.basename(path).replace(".csv", "")
            
            (
                df, data, pre_period, post_period, intervention_start_index
            ) = prep_data_for_causal_impact_analysis(path, Y_col, X_cols, data_type, ci_cfg)
            
            # df_ci_path = f"../output/causal-impact/{run_id}/{data_type}/{session_name}/df_ci.csv"
            df_ci_path = f"../output/causal-impact/{run_id}/{data_type}/{session_name}/r/df_ci_r.csv"
            
            df_ci = pd.read_csv(df_ci_path)
            _data_type = data_type.replace("-", "_")
            file_name = f"fig_ci_{session_name.lower()}_{_data_type}"
            fig_title = f"{session_name} | {title_base}"
            ax_list = [axes[ax_row_idx, j], axes[ax_row_idx+1, j], axes[ax_row_idx+2, j]]
            ax_list = vis_causal_impact_axes(
                ax_list, 
                df_ci, 
                alpha=0.03, 
                title=fig_title, 
                data_type=data_type, 
                ci_cfg=ci_cfg,
                use_r_output=True,
                intervention_start_index=intervention_start_index
            )
            ax = axes[ax_row_idx, j]
            ax.text(
                s=f'{plot_label_list[label_idx]}', x=-0.27, y=1.3, 
                transform=ax.transAxes, fontsize=30, fontweight='bold'
            )
            ax_idx = ax_idx + 1
            label_idx = label_idx + 1

    # to add white space
    for ax in axes[3]:
        ax.axis('off')

    return fig
