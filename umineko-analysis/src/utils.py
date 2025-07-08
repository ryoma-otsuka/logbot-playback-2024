import os
import glob
import numpy as np
# from scipy.interpolate import interp1d
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as tck
import seaborn as sns
from scipy import signal
from sklearn.metrics import jaccard_score, recall_score, precision_score, f1_score
from sklearn.metrics import confusion_matrix
import librosa
import librosa.display
from IPython.display import display
from geopy import distance


palette0 = sns.color_palette(['#E69F00', '#56B4E9', '#009E73', '#F0E442', '#0072B2', '#D55E00', '#CC79A7', '#000000'])
palette2 = sns.color_palette(["#D81B60", "#1E88E5", "#FFC107", "#004D40"])


"""

Data Checking & Extraction

"""
def scan_raw_data_dir(base_dir, test_dir, test_id):
    target_path = f"{base_dir}/{test_dir}/*/logdata/logdata.csv"
    print(f"target_path:\n{target_path}")
    data_dir_list = glob.glob(target_path)
    print("----")
    print(f"Number of scanned csv files: {len(data_dir_list)}")
    for i, data_dir in enumerate(data_dir_list):
        print(data_dir)
    print("----")
    print(f"test_id: {test_id}")
    target_path = f"{base_dir}/{test_dir}/{test_id}*/logdata/24*"
    print(target_path)
    data_dir_list = glob.glob(target_path)
    print(f"N of bin data dirs: {len(data_dir_list)}")
    for i, data_dir in enumerate(data_dir_list):
        
        bin_file_list = glob.glob(f"{data_dir}/*.bin")
        num_bin_files = len(bin_file_list)
        
        small_file_count = 0
        for b, bin_file in enumerate(bin_file_list):
            file_size = os.path.getsize(bin_file)
            if file_size <= 39000: # Normal file size should be larger than 39KB.
                small_file_count += 1



def load_logdata_csv_file(path, sampling_rate):
    df = pd.read_csv(path, low_memory=False)
    display(df.head(5))
    # print(df.columns)

    print(f"len(df): {len(df)}")
    print(f"len(df): {np.round(len(df)/sampling_rate/60/60, 2)} h")
    print(df["rtc_year"].unique())

    return df



def correct_camera_count(df, sampling_rate, show_head_tail=False, test_id="LBP03"):
    display_columns = [
        'rtc_year', 'rtc_month', 'rtc_day', 
        'rtc_hour', 'rtc_min', 'rtc_sec', 'rtc_msec',
        'camera_command', 'camera_recording', 'camera_count']
    n_video_counter = -1
    rec_start_index = 0
    rec_end_index = 0
    camera_count_corrected = False
    
    for i in np.arange(sampling_rate, len(df), sampling_rate):
        camera_count = int(df.at[i, 'camera_count'])
        prev_camera_count = int(df.at[i-sampling_rate, 'camera_count'])
        if camera_count != -1:
            if camera_count != prev_camera_count:
                if prev_camera_count == -1:
                    # update
                    rec_start_index = i
                    n_video_counter += 1
                    print(f"camera_count: {camera_count:02d} | n_video_counter: {n_video_counter:02d} | i: {i}")
                if camera_count != n_video_counter:
                    df.at[i, 'camera_count'] = n_video_counter
                    camera_count_corrected = True
        else: # (camera_count == -1)
            # The moment when the value of camera_count changes from 0 or greater to -1
            if camera_count != prev_camera_count and camera_count_corrected == True:
                rec_end_index = i
                # print(i)
                df_display = df[display_columns].loc[rec_start_index-sampling_rate:rec_end_index]
                df_display = df_display[df_display['rtc_msec'] == 0]
                print("camera_count corrected!!!")
                print(f"i: {i} | camera_count: {prev_camera_count} | n_video_counter: {n_video_counter}")
                if show_head_tail == True:
                    display(df_display.head(3))
                    display(df_display.tail(3))
            else:
                camera_count_corrected = False
        
    # NOTE: for Exception handler
    # Possibly due to an RTC measurement error, some of the data in the beginning of the CSV file was likely overwritten.  
    # This issue occurred during the first video recording session (camera_count = 0) of LBP03,
    # specifically within the first 8 seconds (UTC 19:53:52-59) of the video recording session.  
    # As a result, camera_count = 0 appears twice in separate locations.  
    # With the above algorithm, all values after the second occurrence of camera_count = 0 will be modified.  
    # Therefore, the exception handling below is used to revert the changes.
    if test_id == "LBP03":
        # copy camera_count to _camera_count
        df['_camera_count'] = df['camera_count']
        # modify camera_count values based on _camera_count
        df['camera_count'] = df['_camera_count'].apply(
            lambda x: x if pd.isna(x) or x == -1 else (x - 1 if x > 0 else 0)
        ) 

    return df



def count_camera_recording_and_playback_sessions(df):
    df_audio_file_check = df[ (df['audio_file'] == 1) | (df['audio_file'] == 2)]
    print(np.unique(df['camera_count']))
    if 0 not in np.unique(df['camera_count']): # no video data
        print("No video data recorded.")
    else: # video data recorded 
        num_videos = int(np.max(df_audio_file_check['camera_count'])) + 1 # camera_count = 0, 1, 2, ...., N
        num_hayabusa = len(df_audio_file_check[df_audio_file_check['audio_file'] == 1])
        num_noise = len(df_audio_file_check[df_audio_file_check['audio_file'] == 2])
        print(f"N of total videos: {num_videos}")
        print(f"N of Hayabusa: {num_hayabusa}")
        print(f"N of Noise: {num_noise}")
        print(f"N of cancelled: {num_videos - (num_hayabusa + num_noise)}")


def rtc_data_time_to_timestamp_and_unixtime(df):
    
    # delete columns if exists
    for col in ['datetime_utc', 'datetime_jst', 'unixtime']:
        if col in df.columns:
            df.drop(columns=[col], inplace=True)

    YEAR = df["rtc_year"].iloc[0:1].values[0]
    print(f"YEAR: {YEAR}")
    
    _df = df[df["rtc_year"] != YEAR]
    print(f"len(_df): {len(_df)}")
    if ( len(_df) > 0 ):
        print("Found RTC data bug!!!")
        print(f"strange sec: {int(len(_df) / 25)}")

    
    df = df[df["rtc_year"] == YEAR]
    # for i in range(0, len(df), 1):
    #     if df["rtc_year"].iloc[i:i+1].values[0] != YEAR:
    #         df["rtc_year"].iloc[i:i+1] = YEAR
    # df.loc[df['rtc_year'] != YEAR, 'rtc_year'] = YEAR

    datetime_columns = [
        'rtc_year', 'rtc_month', 'rtc_day', 
        'rtc_hour', 'rtc_min', 'rtc_sec', 'rtc_msec'
    ]
    df_d = df[datetime_columns].copy()
    # display(df_d)
    df_d.columns = [
        'year', 'month', 'day', 
        'hour', 'minute', 'second', 'millisecond', 
        # 'microsecond'
    ]
    datetime_utc = pd.to_datetime(df_d, utc=True)
    df.insert(loc=0, column='datetime_utc', value=datetime_utc)
    datetime_jst = datetime_utc.dt.tz_convert('Asia/Tokyo')
    df.insert(loc=1, column='datetime_jst', value=datetime_jst)
    unixtime = datetime_utc.map(lambda datetime_utc: datetime_utc.timestamp())
    df.insert(loc=0, column='unixtime', value=unixtime)
    return df


def check_duplicated_timestamp(df):
    print("Checking duplicated timestamp")
    duplicated_list = df.duplicated(subset=['unixtime'], keep=False)
    print(f"len(df): {len(df)}")
    print(f"N of duplicates: {np.sum(duplicated_list)}")
    print("------------------------------------")
    duplicated_index_list = []
    for i, duplicated in enumerate(duplicated_list):
        if duplicated == True:
            duplicated_index_list.append(i)
    return duplicated_list, duplicated_index_list


def drop_duplicated_timestamp(df, duplicated_index_list):

    if len(duplicated_index_list) > 0:
        # display(df.head(5))
        df_dropped = df.drop_duplicates(subset=['unixtime'], keep='first')
        duplicated_list = df_dropped.duplicated(subset=['datetime_utc'], keep=False)
        print(f"len(df): {len(df_dropped)}")
        print(f"N of duplicates: {np.sum(duplicated_list)}")
    else:
        df_dropped = df
    
    return df_dropped


def plot_battery_level(df):
    df_battery = df[ (df["rtc_sec"] == 0) & (df["rtc_msec"] == 0) ]
    print(f"len(df_battery): {len(df_battery)}")
    # display(df_battery.head(3))
    # display(df_battery.tail(3))

    elapsed_min_list = np.arange(0, len(df_battery), 1).tolist()
    # df_battery["elapsed_min"] = elapsed_min_list
    MOVING_AVERAGE_MIN = 15
    battery_level_ma = df_battery['battery_level'].rolling(window=MOVING_AVERAGE_MIN, min_periods=1, center=True).mean()
    camera_recordings = df_battery['camera_recording']
    df = pd.DataFrame(
        {
            'elapsed_min': elapsed_min_list,
            'battery_level_ma': battery_level_ma,
            'camera_recording': camera_recordings
        }
    )
    fig, ax = plt.subplots(1, 1, figsize=(12, 6))
    ax.axhline(4.16, c="#1E88E5", label="4.16V (Full)")
    ax.axhline(3.60, c="#FFC107", label="3.60V (Low)")
    # add line
    for i in range(len(df) - 1):
        ax.plot(
            df['elapsed_min'].iloc[i:i+2], df['battery_level_ma'].iloc[i:i+2], 
            color='#555555' if df['camera_recording'].iloc[i] == 0 else '#D81B60',
            linewidth=5.0 if df['camera_recording'].iloc[i] == 0 else 10.0,
            # label="Data"
        )
    ax.set_xticks(np.arange(0, 60*24*3, 60*2)) # tick per 2 hours
    ax.set_xticklabels(np.arange(0, 72, 2))
    # ax.set_xticks(np.arange(-0, 60*24*60, 7200))
    # ax.set_xlim(0-60*1, (40-3)*60) # 00 - 36 h
    # ax.set_xlim(0-60*1, (34-3)*60) # 00 - 30 h
    ax.set_xlim(0-60*1, np.max(elapsed_min_list)+60*1)
    ax.set_yticks(np.arange(2.0, 4.8, 0.2))
    ax.tick_params(axis='both')
    ax.set_ylim(2.65, 4.55)
    ax.set_xlabel("Elapsed Time (h)", labelpad=10)
    ax.set_ylabel("Battery Level (V)", labelpad=10)
    ax.yaxis.set_minor_locator(tck.AutoMinorLocator(2))
    ax.grid(axis='both', which='major', alpha=0.5)
    ax.grid(axis='both', which='minor', alpha=0.5)
    plt.legend(ncols=2, loc="upper right")
    plt.tight_layout()
    plt.show()
    # plt.close()

    return fig



def plot_sd_write_time(df, plot_type='scatter'):
    if plot_type == 'boxplot':
        prev_sd_write_time_ms_s = (df['prev_sd_write_time_ms'].dropna())

        max = np.max(prev_sd_write_time_ms_s)
        min = np.min(prev_sd_write_time_ms_s)
        mean = np.mean(prev_sd_write_time_ms_s)

        print(f"mean: {mean}")
        print(f"min: {min}")
        print(f"max: {max}")

        fig, ax = plt.subplots(1, 1, figsize=(6, 6))
        sns.boxplot(ax=ax, y=prev_sd_write_time_ms_s, color="#56B4E9")
        # ax.axhline(25, label="25 ms", color=palette[2])
        # ax.axhline(40, label="40 ms", color=palette[0])
        ax.axhline(25, label="25 ms", color="#FFC107")
        ax.axhline(40, label="40 ms", color="#D81B60")
        ax.set_ylabel('SD write time (ms)')
        ax.set_yscale('log')
        ax.set_ylim(0.9, 10**3)
        plt.show()
        plt.close()

        return fig

    elif plot_type == 'scatter':
        df = df[(df["rtc_msec"] == 0) ]
        fig, ax = plt.subplots(1, 1, figsize=(12, 4))
        write_index_list = np.arange(0, len(df), 1).tolist()
        ax = sns.scatterplot(
            x=write_index_list, y=df["prev_sd_write_time_ms"], color="#56B4E9", s=15.0,
            linewidth=0, alpha = 0.75)
        # ax.axhline(25, label="25 ms", color=palette[2])
        # ax.axhline(40, label="40 ms", color=palette[0])
        ax.axhline(25, label="25 ms", color="#FFC107")
        ax.axhline(40, label="40 ms", color="#D81B60")
        ax.set_xlabel("Index (sec)", labelpad=10)
        ax.set_ylabel("SD process time (ms)", labelpad=10)
        ax.set_xticks(np.arange(0, 60*60*48, 60*60*5))
        # ax.set_xlim((60*60*(0-1), 60*60*(35+1))) # max of 36 hours
        ax.set_xlim((60*60*(0-1), 60*60*(30+1))) # max of 30 hours
        ax.set_yscale('log')
        ax.set_ylim(0.9, 10**3) # max of 33 hours
        ax.legend(ncol=2)
        plt.show()
        plt.close()
        return fig


def sd_write_delay_check(df):

    df_delay_count = df[ (df["delay_occurred_counter"] > 0) & (df["rtc_msec"] == 0) ]
    print(f"len(df_delay) total: {len(df_delay_count)} (s)")

    df_delay_count = df[ (df["delay_occurred_counter"] > 0) & (df["rtc_msec"] == 0) &  (df["camera_recording"] == 1) ]
    print(f"len(df_delay) during video recording: {len(df_delay_count)} (s)")

    threshold_list = [40, 35, 30, 25, 20]
    for i, threshold in enumerate(threshold_list):
        df_sd_delay = df[ (df["prev_sd_write_time_ms"] > threshold) & (df["rtc_msec"] == 0) ]
        print(f"> {threshold} ms: {len(df_sd_delay)} (s)")


def plot_rtc_and_gps_data_gap(df):
    df_0s = df[ (df["rtc_sec"] == 0) & (df["rtc_msec"] == 0) ] # msec -> sec level
    df_0s_fixed = df_0s[df_0s['fix_type'] >= 2.0]
    sec_diff_list = list(df_0s_fixed['rtc_sec'] - df_0s_fixed['gps_sec'])
    abs_sec_diff_list = []
    for i, sec_diff in enumerate(sec_diff_list): 
        if sec_diff_list[i] == 59 or sec_diff_list[i] == -59:
            abs_sec_diff_list.append(1)
        elif sec_diff_list[i] == 58 or sec_diff_list[i] == -58:
            abs_sec_diff_list.append(2)
        # elif sec_diff_list[i] < 0:
        #     abs_sec_diff_list.append(-sec_diff)
        else:
            abs_sec_diff_list.append(sec_diff)
    print(f"len(df_0s): {len(df_0s)} ({np.round(len(df_0s) / 60, 2)} h)")
    print(f"len(df_0s_fixed): {len(df_0s_fixed)}")
    print(f"np.max(abs_sec_diff_list): {np.max(abs_sec_diff_list)}")
    
    t = np.arange(0, len(df_0s_fixed), 1)
    y = abs_sec_diff_list
    fig, ax = plt.subplots(1, 1, figsize=(20, 5))
    ax = sns.lineplot(ax=ax, x=t, y=y, alpha=0.9, color="#56B4E9", linewidth=5.0)
    # ax = sns.scatterplot(ax=ax, x=t, y=y, alpha=0.9, color=palette[1])
    # ax = sns.stripplot(ax=ax, x=t, y=y, alpha=0.9, color=palette[1])
    # ax.plot(t, y, alpha=0.9, color=palette[1])
    # ax.set_xticks(np.arange(-0, 60*24*3, 60*2)) # tick per 2 days
    # ax.set_xticklabels(np.arange(0, 72, 2))
    ax.set_yticks(np.arange(-10, 10, 1))
    ax.set_ylim(-0.5, 3.5)
    ax.set_xlabel("Fixed GPS data count (at hh:mm:00:000)", labelpad=15)
    ax.set_ylabel("Diff", labelpad=15)
    ax.grid()
    plt.show()
    plt.close()

    return fig


def extract_camera_recording_session(
        df, 
        use_columns, 
        ffill_columns,
        ):
    '''
    Extract data only when the logger was recording videos
    '''
    df = df[use_columns]
    df.loc[:, ffill_columns] = df[ffill_columns].fillna(method='ffill')
    df = df[ (df['camera_command'] == 1) | (df['camera_recording'] == 1)]
    return df


def identify_audio_file(df):
    if np.unique(df['audio_file']).size >= 2:
        audio_file = int(np.unique(df['audio_file'])[1])
    else:
        audio_file = int(np.unique(df['audio_file'])[0])
    # print(audio_file)
    if audio_file == 1:
        audio_file_name = "Predator call"  
    elif audio_file == 2: 
        audio_file_name = "White noise"
    else:
        audio_file_name = "None"
    return audio_file, audio_file_name


def get_speaker_turn_on_idx(df):
    speaker_on_list = list(df['speaker_on'])
    # index where speaker_on = 1 (for the first time in the data frame)
    idx_list = [i for i, x in enumerate(speaker_on_list) if x == 1]
    idx = idx_list[0] if len(idx_list) > 0 else 0
    return idx


def modify_speaker_turn_on_idx(idx, data_type='gps'):
    if data_type=='gps':
        idx = idx + 1
    elif data_type=='imu':
        idx = int(idx + 25 * 1)
    return idx


def calc_speed(df):
    '''
    Calculate ground speed (m/s) and add 'speed' columns
    '''
    # set the initial ground speed as zero
    speeds_geodesic = [0]  
    speeds_distance = [0]
    for i in range(1, len(df)):
        coord1 = (df.at[i-1, 'latitude'], df.at[i-1, 'longitude'])
        coord2 = (df.at[i, 'latitude'], df.at[i, 'longitude'])
        # calculate travel distance (m)
        distance_geodesic = distance.geodesic(coord1, coord2).meters  
        distance_distance = distance.distance(coord1, coord2).meters
        # 1 s update time -> distance (m) = ground speed (m/s)
        speeds_geodesic.append(distance_geodesic)
        speeds_distance.append(distance_distance)
    df.insert(len(df.columns), 'speed_geodesic_m_s', speeds_geodesic)
    df.insert(len(df.columns), 'speed_geodesic_km_h', df['speed_geodesic_m_s']*3.6)
    df.insert(len(df.columns), 'speed_distance_m_s', speeds_distance)
    df.insert(len(df.columns), 'speed_distance_km_h', df['speed_distance_m_s']*3.6)

    # calculate abs_speed_diff
    diff_distance_list = [0]
    abs_diff_distance_list = [0]
    cum_distance_list = [0]
    cum_distance = 0
    for i in list(np.arange(1, len(df), 1)):
        cum_distance = cum_distance + df.loc[i, 'speed_distance_m_s']
        cum_distance_list.append(cum_distance)
        diff = df.loc[i, 'speed_distance_m_s'] - df.loc[i-1, 'speed_distance_m_s']
        diff_distance_list.append(diff)
        abs_diff_distance_list.append(np.abs(diff))
    df.insert(len(df.columns), 'cum_distance_m', cum_distance_list)
    df.insert(len(df.columns), 'diff_distance_m', diff_distance_list)
    df.insert(len(df.columns), 'abs_diff_distance_m', abs_diff_distance_list)
    df.insert(len(df.columns), 'abs_speed_diff', abs_diff_distance_list)

    return df


def convert_speed_m_per_sec_to_km_per_hour(speed_m_per_sec):
    return (speed_m_per_sec * 60 * 60) / 1000 # m/s -> km/h (* 3.6)


def calc_turning_angle(df):
    # Calculate angle changes for each time step (1Hz) and add the data as 'angle_change' column
    turning_angles = [0, 0]  # Set the first two angle changes to 0
    for i in range(2, len(df)):
        # Fetch the coordinates (latitude and longitude)
        # at 2 seconds ago, 1 seconds ago, and the current time
        lat1, lon1 = df.at[i-2, 'latitude'], df.at[i-2, 'longitude']
        lat2, lon2 = df.at[i-1, 'latitude'], df.at[i-1, 'longitude']
        lat3, lon3 = df.at[i, 'latitude'], df.at[i, 'longitude']
        
        # Calculate the direction vector
        vector1 = np.array([lat2 - lat1, lon2 - lon1])
        vector2 = np.array([lat3 - lat2, lon3 - lon2])
        
        # Calculate the dot (inner) product of vectors
        dot_product = np.dot(vector1, vector2)

        # Calculate the norm (magnitude) product of vectors
        norm1 = np.linalg.norm(vector1)
        norm2 = np.linalg.norm(vector2)
        
        # Calculate the cosine similarity (cos theta)
        # and obtain the angle (theta)
        if norm1 * norm2 == 0:
            # angle = np.nan
            angle = 0
        else:
            cos_theta = dot_product / (norm1 * norm2)
            angle = np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))  # clip cos_theta (-1 to +1) for safety
        
        turning_angles.append(angle)
    df.insert(len(df.columns), 'turning_angle', turning_angles)
    
    # Set the turning angle to 0, if the ground speed is less than 10 km/h,
    # to address GPS data noise (measurement errors) when the bird was not moving.
    df.loc[df['speed_distance_km_h'] < 10, 'turning_angle'] = 0

    return df


def plot_speed_and_truning_angle(
    path_list,
    value_type = 'absolute', # 'absolute' or 'relative'
    save_dir = None,
    plot_sec_length = 30
):
    
    for i, path in enumerate(path_list):
        df = pd.read_csv(path)
        _session_name = os.path.basename(path)
        audio_file, audio_file_name = identify_audio_file(df)

        idx = get_speaker_turn_on_idx(df)
        # print(f"idx: {idx}")
        
        if idx == 0:
            print(f"{i:02d} | {_session_name} -> skipped.")
            continue
        
        if value_type == 'relative':
            speed_before_mean = np.mean(df.loc[idx-plot_sec_length:idx, 'speed_distance_km_h'])
            turning_angle_before_mean = np.mean(df.loc[idx-plot_sec_length:idx, 'turning_angle'])
            print(f"speed_before_mean: {speed_before_mean:.2f}")
            print(f"turning_angle_before_mean: {turning_angle_before_mean:.2f}")

            _speed = df['speed_distance_km_h'] - speed_before_mean
            _turning_angle = df['turning_angle'] - turning_angle_before_mean
            yticks_ax1 = np.arange(-100, 100, 20)
            yticks_ax2 = np.arange(-100, 100, 20)
            ylim_ax1 = (-45, 45)
            ylim_ax2 = (-45, 45)
            ylabel_ax1, ylabel_ax2 = "Relative speed (km/h)", "Relative turning angle (°)"

        elif value_type == 'absolute': # 'absolute'
            _speed = df['speed_distance_km_h']
            _turning_angle = df['turning_angle']
            yticks_ax1 = np.arange(-30, 180, 30)
            yticks_ax2 = np.arange(-30, 180, 30)
            ylim_ax1 = (-5, 70)
            ylim_ax2 = (-5, 70)
            ylabel_ax1, ylabel_ax2 = "Speed (km/h)", "Turning angle (°)"
            
        fig, ax1 = plt.subplots(1, 1, figsize=(18, 5))
        t = np.arange(0, len(df), 1)
        t = df['_program_index']
        lines1 = sns.lineplot(
            ax=ax1, 
            x=t,
            y=_speed, 
            marker='o', 
            markersize=10, 
            dashes=False, 
            # scatter_kws={'s': 100},
            color=palette0[0], 
            label='speed'
        )
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel(ylabel_ax1, labelpad=10)
        
        xticks = np.arange(-100, 100, 5)
        ax1.set_xticks(xticks)
        ax1.set_xlim(-plot_sec_length-1.5, plot_sec_length+4+1.5)
        
        ax1.set_yticks(yticks_ax1)
        ax1.yaxis.set_minor_locator(tck.AutoMinorLocator(2))
        ax1.set_ylim(ylim_ax1)

        ax1.xaxis.set_minor_locator(tck.AutoMinorLocator(5))
        
        ax1.grid(which="major", alpha=0.4)
        ax1.grid(which="minor", alpha=0.2)
        title_0 = os.path.basename(path).replace("_", " | ").replace(".csv", "")
        ax1.set_title(f"{title_0} | {audio_file_name}", fontweight='normal', pad=15)
        

        # The second y axis (angle)
        ax2 = ax1.twinx()
        lines2 = sns.lineplot(
            ax=ax2,
            x=t, 
            y=_turning_angle, 
            marker='o', 
            markersize=10, 
            dashes=False, 
            color=palette0[1], 
            label='turning angle'
        )
        ax2.set_ylabel(ylabel_ax2, rotation=270, labelpad=40)

        ax2.set_yticks(yticks_ax2)
        ax2.set_ylim(ylim_ax2)
        
        # legend
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.get_legend().remove()
        ax2.legend(lines1 + lines2, labels1 + labels2, ncol=2, loc='upper right')
        
        ax1.axvspan(0, 4.2, color="#808080", alpha=0.25)
        # if idx > 0:
        #     ax1.axvspan(t[idx], t[idx+4]+0.2, color="#808080", alpha=0.25)
            
        
        # add baseline information for relative plot
        if value_type == 'relative':
            ax1.text(
                s=f"baseline speed = {speed_before_mean:.2f}\nbaseline turning angle = {turning_angle_before_mean:.2f}", 
                # x=idx-plot_sec_length, 
                x=-plot_sec_length, 
                y=ylim_ax1[0] + ylim_ax1[1]*0.2,
                linespacing=1.5,
                fontsize=20,
                bbox=dict(facecolor='white', alpha=0.6, pad=5)
            )

        if save_dir is None:
            plt.show()
        else:
            filename = f"{os.path.basename(path)[:5]}_{os.path.basename(path)[6:9]}"
            # filename = filename.lower()
            fig.savefig(f"{save_dir}/{filename}.png", 
                        bbox_inches="tight", 
                        dpi=72,
                        pad_inches=0.25, 
                        transparent=False)
            print(f"{i:02d} | {_session_name} -> saved.")

        plt.close()


def plot_abs_speed_diff(
    path_list,
    value_type = 'abs_diff_distance_m',
    save_dir = None,
    plot_sec_length = 20
):
    
    for i, path in enumerate(path_list):
        df = pd.read_csv(path)
        _session_name = os.path.basename(path)
        audio_file, audio_file_name = identify_audio_file(df)

        y_values = df[value_type]
        # ylabel = "Abs. Speed Diff."
        ylabel = "AD-Speed"
        yticks = np.arange(-100, 100, 5)
        ylim = (0-3, 15+3)

        idx = get_speaker_turn_on_idx(df)
        # print(f"idx: {idx}")
        if idx == 0:
            print(f"{i:02d} | {_session_name} -> skipped") 
            continue

        fig, ax = plt.subplots(1, 1, figsize=(18, 5))
        t = np.arange(0, len(df), 1)
        t = df['_program_index']
        lines = sns.lineplot(
            ax=ax, 
            x=t,
            y=y_values, 
            marker='o', markersize=10, dashes=False, 
            # scatter_kws={'s': 100},
            color="#FFC107", 
            # label='ad-speed'
        )
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(ylabel, labelpad=10)

        xticks = np.arange(-100, 100, 5)
        ax.set_xticks(xticks)
        ax.set_xlim(-plot_sec_length-1.5, plot_sec_length+4+1.5)
        ax.xaxis.set_minor_locator(tck.AutoMinorLocator(5))
        
        ax.set_yticks(yticks)
        ax.yaxis.set_minor_locator(tck.AutoMinorLocator(2))
        ax.set_ylim(ylim)

        ax.grid(which="major", alpha=0.4)
        ax.grid(which="minor", alpha=0.2)
        title_0 = os.path.basename(path).replace("_", " | ").replace(".csv", "")
        ax.set_title(f"{title_0} | {audio_file_name}", fontweight='normal', pad=15)
        
        if idx > 0:
            ax.axvspan(0, 4.2, color="#808080", alpha=0.25)

        if save_dir is None:
            plt.show()
        else:
            filename = f"{os.path.basename(path)[:5]}_{os.path.basename(path)[6:9]}"
            # filename = filename.lower()
            fig.savefig(f"{save_dir}/{filename}.png", 
                        bbox_inches="tight", 
                        dpi=72,
                        pad_inches=0.25, 
                        transparent=False)
            print(f"{i:02d} | {_session_name} -> saved.")

            print(f"{i:02d} | {_session_name} -> saved") 

        plt.close()


def prep_relative_speed_and_turning_angle_df(
    path_list,
    plot_sec_length = 30
):
    df_all = pd.DataFrame()

    for i, path in enumerate(path_list):
        test_id = os.path.basename(path)[:5]
        df = pd.read_csv(path)
        audio_file, audio_file_name = identify_audio_file(df)

        idx = get_speaker_turn_on_idx(df)
        # print(f"idx: {idx}")
        
        if idx == 0:
            continue
        
        speed_before_mean = np.mean(df.loc[idx-plot_sec_length:idx, 'speed_distance_km_h'])
        turning_angle_before_mean = np.mean(df.loc[idx-plot_sec_length:idx, 'turning_angle'])
        # print(f"speed_before_mean: {speed_before_mean:.2f}")
        # print(f"turning_angle_before_mean: {turning_angle_before_mean:.2f}")

        _speed = df['speed_distance_km_h'] - speed_before_mean
        _turning_angle = df['turning_angle'] - turning_angle_before_mean
        df.insert(len(df.columns), 'speed', df['speed_distance_km_h'])
        df.insert(len(df.columns), 'relative_speed', _speed)
        df.insert(len(df.columns), 'relative_turning_angle', _turning_angle)

        df.insert(len(df.columns), 'audio_file_name', [audio_file_name]*len(df))
        # df.insert(len(df.columns), 'test_id', [test_id]*len(df))
        colums = [
            'file_name',
            'test_id', 'session_id', '_program_index', 'program_index',
            'datetime_jst', 'camera_count', 'audio_file_name', 'speaker_on',  
            'speed', 'turning_angle',
            'relative_speed', 'relative_turning_angle'
        ]
        df = df[colums]
        # print(len(df))
    
        df_all = pd.concat([df_all, df])

    df_all.insert(0, 'old_index', df_all.index)
    df_all.reset_index(drop=True, inplace=True)

    return df_all


def plot_relative_value_per_audio_file_type(
    df, 
    value_type='speed', 
    plot_sec_length=30,
    highlight_dict=None):

    if value_type == 'speed':
        y_column = 'relative_speed'
        ylabel = 'Relative speed (km/h)'
    elif value_type == 'turning_angle':
        y_column = 'relative_turning_angle'
        ylabel = 'Relative turning angle (°)'

    GRIDSPEC_KW = {'width_ratios': [1, 1], 'wspace': 0.3, 'hspace': 0.7}
    fig, axes = plt.subplots(1, 2, figsize=(20, 4), gridspec_kw=GRIDSPEC_KW)
    ax_list = axes.flatten().tolist()
    audio_file_name_list = ['Predator call', 'White noise']
    title_list = ['Predator', 'Noise']
    for a, ax in enumerate(ax_list):
        _df = df[df['audio_file_name'] == audio_file_name_list[a]]
        ax = sns.lineplot(
            ax=ax, 
            data=_df, 
            x='_program_index', 
            y=y_column, 
            units="file_name",
            # color=palette0[a],
            # color=palette0[1],
            color="#333333",
            estimator=None, linewidth=2, alpha=0.5
        )

        if highlight_dict is not None:
            first_key = list(highlight_dict.keys())[0]
            df_hl = _df[_df[f'{first_key}'].isin(highlight_dict[f'{first_key}'])]
            ax = sns.lineplot(
                ax=ax, 
                data=df_hl, 
                x='_program_index', 
                y=y_column, 
                units="file_name",
                # color=palette0[a],
                color=palette0[1],
                # color=palette0[0],
                # color="#333333",
                estimator=None, linewidth=2, alpha=0.9
            )

        ax.set_xticks(np.arange(-100, 100, 10))
        ax.set_xlim(-plot_sec_length-1, plot_sec_length+2)
        ax.set_yticks(np.arange(-180, 180, 30))
        ax.set_ylim(-65, 65)
        ax.grid(which='major')
        ax.set_xlabel('Time (s)', labelpad=10)
        ax.set_ylabel(ylabel, labelpad=10)
        SPAN_COLOR = "#808080"
        ax.axvspan(0, 4.2, color=SPAN_COLOR, alpha=0.20)
        ax.set_title(title_list[a], pad=10)
    plt.show()
    plt.close()
    return fig


def plot_acc_data_v5(
    ax, df, t, 
    speaker_turn_on_idx, idx_0, idx_1, color_list,
    filter_type=None
):
    
    LINE_WIDTH = 2.5
    ALPHA = 0.8
    
    if filter_type == 'ma': # Moving average
        acc_x = df['acc_x_ma']
        acc_y = df['acc_y_ma']
        acc_z = df['acc_z_ma']
    elif filter_type == 'butter': # Butterworth filter
        sampling_rate = 25
        # cutoff_freq = 3
        cutoff_freq = 2
        nyquist_freq = 0.5 * sampling_rate
        normal_cutoff = cutoff_freq / nyquist_freq
        b, a = signal.butter(N=4, Wn=normal_cutoff, btype='low')
        acc_x = signal.filtfilt(b, a, df['acc_x'])
        acc_y = signal.filtfilt(b, a, df['acc_y'])
        acc_z = signal.filtfilt(b, a, df['acc_z'])
    else: # None
        acc_x = df['acc_x']
        acc_y = df['acc_y']
        acc_z = df['acc_z']
    

    ax = sns.lineplot(
        ax=ax, x=t, y=acc_x, 
        color=color_list[0], 
        linewidth=LINE_WIDTH,
        alpha=ALPHA,
        label="x"
    )
    ax = sns.lineplot(
        ax=ax, x=t, y=acc_y, 
        color=color_list[2], 
        linewidth=LINE_WIDTH,
        alpha=ALPHA,
        label="y"
    )
    ax = sns.lineplot(
        ax=ax, x=t, y=acc_z, 
        color=color_list[1], 
        linewidth=LINE_WIDTH,
        alpha=ALPHA,
        label="z"
    )
    
    ax.set_yticks(np.arange(-10.0, 10.0, 2.0))
    # ax.set_ylim(-4.5, 4.5)
    ax.set_ylim(-8.5, 8.5)
    ax.set_ylabel("Acceleration (g)", labelpad=20)
    plt.rcParams['legend.handlelength'] = 1.5
    ax.legend(ncol=3, loc="lower right")

    return ax


def calc_lowpass_filtered_gyro_data(df, filter_type='butter'):
    sampling_rate = 25
    # cutoff_freq = 3
    cutoff_freq = 2
    nyquist_freq = 0.5 * sampling_rate
    normal_cutoff = cutoff_freq / nyquist_freq
    b, a = signal.butter(N=4, Wn=normal_cutoff, btype='low')
    gyro_x_filtered = signal.filtfilt(b, a, df['gyro_x'])
    gyro_y_filtered = signal.filtfilt(b, a, df['gyro_y'])
    gyro_z_filtered = signal.filtfilt(b, a, df['gyro_z'])

    df.insert(len(df.columns), 'gyro_x_lowpass', gyro_x_filtered)
    df.insert(len(df.columns), 'gyro_y_lowpass', gyro_y_filtered)
    df.insert(len(df.columns), 'gyro_z_lowpass', gyro_z_filtered)

    return df


def calc_angle_change_using_filtered_gyro_and_ma_acc_data(df):
    sampling_rate = 25

    # Gyro data after applying the low pass filter
    gyro_x_filtered = df['gyro_x_lowpass']
    gyro_y_filtered = df['gyro_y_lowpass']
    gyro_z_filtered = df['gyro_z_lowpass']

    # Integration of angle changes (initial value set to 0)
    dt = 1.0 / sampling_rate  # update time
    angle_x = np.cumsum(gyro_x_filtered * dt)
    angle_y = np.cumsum(gyro_y_filtered * dt)
    angle_z = np.cumsum(gyro_z_filtered * dt)
    df.insert(len(df.columns), 'angle_x', angle_x)
    df.insert(len(df.columns), 'angle_y', angle_y)
    df.insert(len(df.columns), 'angle_z', angle_z)
    
    # Calculate the gravity component using acceleration data
    acc_x_ma = df['acc_x_ma']
    acc_y_ma = df['acc_y_ma']
    acc_z_ma = df['acc_z_ma']
    gravity_x = np.sin(np.arctan2(acc_y_ma, acc_z_ma))
    gravity_y = np.sin(np.arctan2(acc_x_ma, acc_z_ma))

    # Create an indicator by combining angle changes after low-pass filtering and gravity components
    combined_angle_change = np.sqrt(df['angle_x']**2 + df['angle_y']**2 + df['angle_z']**2)
    gravity_adjusted_angle = np.arctan2(gravity_x, gravity_y)
    df.insert(len(df.columns), 'combined_angle_change', combined_angle_change)
    df.insert(len(df.columns), 'gravity_adjusted_angle', gravity_adjusted_angle)

    # Additional smoothing
    smoothed_angle_change = df['combined_angle_change'].rolling(window=10, min_periods=1, center=True).mean()
    smoothed_gravity_angle = df['gravity_adjusted_angle'].rolling(window=10, min_periods=1, center=True).mean()
    df.insert(len(df.columns), 'smoothed_angle_change', smoothed_angle_change)
    df.insert(len(df.columns), 'smoothed_gravity_angle', smoothed_gravity_angle)

    return df


def plot_gyro_data_v5(
    ax, df, t, 
    speaker_turn_on_idx, idx_0, idx_1, color_list,
    filter_type=None
):
    LINE_WIDTH = 2.5
    ALPHA = 0.8
    
    if filter_type == 'ma': # Moving average
        gyro_x = df['gyro_x_ma']
        gyro_y = df['gyro_y_ma']
        gyro_z = df['gyro_z_ma']
    elif filter_type == 'butter': # Butterworth filter
        gyro_x = df['gyro_x_lowpass']
        gyro_y = df['gyro_y_lowpass']
        gyro_z = df['gyro_z_lowpass']
    else: # None
        gyro_x = df['gyro_x']
        gyro_y = df['gyro_y']
        gyro_z = df['gyro_z']


    ax = sns.lineplot(
        ax=ax, x=t, y=gyro_x, 
        color=color_list[0], 
        linewidth=LINE_WIDTH,
        alpha=ALPHA,
        label="x"
    )
    ax = sns.lineplot(
        ax=ax, x=t, y=gyro_y, 
        color=color_list[2], 
        linewidth=LINE_WIDTH,
        alpha=ALPHA,
        label="y"
    )
    ax = sns.lineplot(
        ax=ax, x=t, y=gyro_z, 
        color=color_list[1], 
        linewidth=LINE_WIDTH,
        alpha=ALPHA,
        label="z"
    )

    ax.set_yticks(np.arange(-180.0, 181.0, 90.0))
    # ax.set_xlim(275-(sampling_rate*11)-20, 275+(sampling_rate*12)+20)
    ax.set_ylim(-210, 210)
    ax.set_ylabel("Gyro ($^\circ$/s)")
    plt.rcParams['legend.handlelength'] = 1.5
    ax.legend(ncol=3, loc="lower right")

    return ax


def calc_odba_and_vedba(df, filter_type=None):
    ACC_SAMPLING_RATE = 25
    if filter_type is None or filter_type == "ma":
        acc_x_st = df['acc_x_ma']
        acc_y_st = df['acc_y_ma']
        acc_z_st = df['acc_z_ma']
    df.insert(len(df.columns), 'acc_x_dy', df['acc_x'] - acc_x_st)
    df.insert(len(df.columns), 'acc_y_dy', df['acc_y'] - acc_y_st)
    df.insert(len(df.columns), 'acc_z_dy', df['acc_z'] - acc_z_st)

    # e.g., Wilson et al., 2020 Journal of Animal Ecology
    # ODBA -> L1 norm
    df.insert(
        len(df.columns), 
        'ODBA', 
        np.abs(df['acc_x_dy']) + np.abs(df['acc_y_dy']) + np.abs(df['acc_z_dy'])
    )
    # VeDBA -> L2 norm
    df.insert(
        len(df.columns), 
        'VeDBA', 
        np.sqrt(df['acc_x_dy']**2 + df['acc_y_dy']**2 + df['acc_z_dy']**2)
    )
    
    # Smoothed VeDBA (1, 2, 3 seconds)
    df.insert(
        len(df.columns), 
        'smoothed_VeDBA_1s', 
        df['VeDBA'].rolling(window=1*ACC_SAMPLING_RATE, min_periods=1, center=False).mean()
    )
    df.insert(
        len(df.columns), 
        'smoothed_VeDBA_2s', 
        df['VeDBA'].rolling(window=2*ACC_SAMPLING_RATE, min_periods=1, center=False).mean()
    )
    df.insert(
        len(df.columns), 
        'smoothed_VeDBA_3s', 
        df['VeDBA'].rolling(window=3*ACC_SAMPLING_RATE, min_periods=1, center=False).mean()
    )

    return df


def plot_odba_vedba_data_v5(
    ax, df, t, 
    speaker_turn_on_idx, idx_0, idx_1, color_list,
    dba_type='VeDBA', # save as data_type
):
    
    LINE_WIDTH = 2.5
    ALPHA = 0.9
    
    y_val = df[dba_type]
    ax = sns.lineplot(
        ax=ax, x=t, y=y_val, 
        color=color_list[1], # blue
        linewidth=LINE_WIDTH,
        alpha=ALPHA,
    )
    
    if "smoothed" in dba_type:
        y_label = dba_type.replace("smoothed", "s").replace("_", "-")
    else:
        y_label = dba_type
    ax.set_ylabel(y_label)

    ax.set_yticks(np.arange(-10.0, 10.0, 2.0))
    ax.set_ylim(-0.5, 4.5)

    return ax


def plot_angle_data_v5(
    ax, df, t, 
    speaker_turn_on_idx, idx_0, idx_1, color_list,
    data_type='combined_angle_change',
):
    
    LINE_WIDTH = 2.5
    ALPHA = 0.9
    
    y_val = df[data_type]
    # print(y_val)
    ax = sns.lineplot(
        ax=ax, x=t, y=y_val, 
        color="#333333", # black
        linewidth=LINE_WIDTH,
        alpha=ALPHA,
        label="x"
    )
    
    x_interval_sec = 4
    SR = 25

    xticks = np.arange(idx_0, idx_1 + 2*SR, SR*x_interval_sec)
    xticklabels = np.arange(idx_0/SR, ((idx_1) / SR) + 2, 1*x_interval_sec).astype(int)
    xticklabels = xticklabels - int(speaker_turn_on_idx / SR)
    
    if 0 not in xticklabels:
        closest_to_zero_idx = np.argmin(np.abs(xticklabels))
        shift = -xticklabels[closest_to_zero_idx]
        xticks = xticks + shift * SR
        xticklabels = xticklabels + shift

    ax.set_xticks(xticks)
    ax.set_xticklabels(xticklabels)
    ax.set_xlim(idx_0 - 1*SR+1, idx_1+1*SR-1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(f"{data_type}")
    ax.legend(ncol=3, loc="lower right")

    return ax

def plot_imu_data_v5(
    df, 
    ax, 
    session_name=None,
    data_type='acc', # 'acc' / 'gyro' / 'ODBA' / 'VeDBA' / 'smoothed_VeDBA_2s' / 'smoothed_VeDBA_3s'
    filter_type=None,
    sampling_rate=25, 
    plot_before_sec=20,
    plot_after_sec=20,
    title=True,
    x_interval_sec=5,
):
    color_list = ['#D81B60', '#1E88E5', '#FFC107']

    # Note: 
    # No slide rquired for logbot-v5 umineko 2024 version 
    # All data at time t were copied and saved at time t+1
    slide = 0
    # slide = sampling_rate*1
    
    speaker_turn_on_idx = get_speaker_turn_on_idx(df)
    # speaker_turn_on_idx = modify_speaker_turn_on_idx(
    #     speaker_turn_on_idx, 
    #     data_type='imu'
    # )
    speaker_turn_on_sec = int(speaker_turn_on_idx/sampling_rate)
    # print(f"speaker_turn_on_idx: {speaker_turn_on_idx}")
    # print(f"speaker_turn_on_sec: {speaker_turn_on_sec}")
    

    t = np.arange(0, len(df), 1)

    if speaker_turn_on_idx == 0:
        print("Playback cancelled !")
        # df_tmp = df[0+slide:0+slide+plot_time_window]
        # t_tmp = t[0:0+plot_time_window]
        # ax = plot_acc_data_v5(ax, df_tmp, t_tmp, plot_time_window, color_list)
        # return ax
        # return None
    
    idx_0 = slide + speaker_turn_on_idx - plot_before_sec * sampling_rate # plot start index
    idx_1 = slide + speaker_turn_on_idx + plot_after_sec * sampling_rate  # plot end index

    # df_tmp = df[idx_0:idx_1]
    # t_tmp = t[idx_0:idx_1]
    # print(f"idx_0: {idx_0}")
    # print(f"idx_1: {idx_1}")
    # print(len(df_tmp))
    # print(len(t_tmp))
    df_tmp = df
    t_tmp = t
    
    if data_type=='acc':
        ax = plot_acc_data_v5(
            ax, df_tmp, t_tmp, 
            speaker_turn_on_idx, idx_0, idx_1, color_list,
            filter_type=filter_type
        )
    elif data_type=="gyro":
        ax = plot_gyro_data_v5(
            ax, df_tmp, t_tmp, 
            speaker_turn_on_idx, idx_0, idx_1, color_list,
            filter_type=filter_type
        )
    elif data_type in ["ODBA", "VeDBA", 
                       "smoothed_VeDBA_1s", 
                       "smoothed_VeDBA_2s", 
                       "smoothed_VeDBA_3s"]:
        if "ODBA" not in df_tmp.columns and "VeDBA" not in df_tmp.columns:
            df_tmp = calc_odba_and_vedba(df_tmp, filter_type)
        ax = plot_odba_vedba_data_v5(
            ax, df_tmp, t_tmp, 
            speaker_turn_on_idx, idx_0, idx_1, color_list,
            dba_type=data_type
        )
    elif data_type in ["combined_angle_change", "gravity_adjusted_angle", 
                       "smoothed_angle_change", "smoothed_gravity_angle"]:
        ax = plot_angle_data_v5(
            ax, df_tmp, t_tmp, 
            speaker_turn_on_idx, idx_0, idx_1, color_list,
            data_type=data_type
        )

    # Audio playback timing
    if speaker_turn_on_idx > 0:
        SPAN_COLOR = "#808080"
        ax.axvspan(
            # t_tmp[speaker_turn_on_idx - idx_0], 
            # t_tmp[speaker_turn_on_idx + (sampling_rate)*4 + 5 - idx_0], # 4.2 s 
            t_tmp[speaker_turn_on_idx], 
            t_tmp[speaker_turn_on_idx + (sampling_rate)*4 + 5], # 4.2 s 
            color=SPAN_COLOR, alpha=0.20
        )

    # title
    if title == True:
        ax.set_title(session_name, fontweight='normal', pad=15)

    # x axis
    # x_interval_sec = 10
    SR = 25
    xticks = np.arange(idx_0, idx_1 + 2*SR, SR*x_interval_sec)
    xticklabels = np.arange(idx_0/SR, ((idx_1) / SR) + 2, 1*x_interval_sec).astype(int)
    xticklabels = xticklabels - int(speaker_turn_on_idx / SR)
    if 0 not in xticklabels:
        closest_to_zero_idx = np.argmin(np.abs(xticklabels))
        shift = -xticklabels[closest_to_zero_idx]
        xticks = xticks + shift * SR
        xticklabels = xticklabels + shift
    ax.set_xticks(xticks)
    ax.set_xticklabels(xticklabels)
    ax.set_xlim(idx_0 - 1*SR+1, idx_1+1*SR-1)
    ax.set_xlabel("Time (s)")
    
    # grid
    ax.xaxis.set_minor_locator(tck.AutoMinorLocator(x_interval_sec))
    ax.yaxis.set_minor_locator(tck.AutoMinorLocator(2))
    ax.grid(axis='both', which='major', alpha=0.4)
    ax.grid(axis='both', which='minor', alpha=0.2)

    return ax


def vis_imu_data_v5(
    path_list, 
    data_type='acc',
    save_dir=None,
    filter_type=None,
    before_sec=20,
    title=True,
):
    # visualization
    SAMPLING_RATE = 25
    BEFORE_SEC = before_sec
    # pd.option_context('mode.use_inf_as_na', True)
    for i, path in enumerate(path_list):
        _session_name = os.path.basename(path)
        session_name = f"{os.path.basename(path)[:5]} | {os.path.basename(path)[6:9]}"
        df = pd.read_csv(path_list[i])
        speaker_turn_on_idx = get_speaker_turn_on_idx(df)
        if speaker_turn_on_idx == 0:
            print(f"{i:02d} | {_session_name} -> skipped.")
            continue # skip
        audio_file, audio_file_name = identify_audio_file(df)
        # print(audio_file)
        
        if BEFORE_SEC < 15:
            fig_size = (18*0.6, 5*0.6)
        else:
            fig_size = (18, 5)        
        fig, ax = plt.subplots(1, 1, figsize=fig_size)
        ax = plot_imu_data_v5(
            df, 
            ax, 
            session_name=f"{session_name} | {audio_file_name}", 
            data_type=data_type,
            filter_type=filter_type, 
            sampling_rate=SAMPLING_RATE, 
            plot_before_sec=BEFORE_SEC, 
            plot_after_sec=BEFORE_SEC+5,
            title=title,
        )

        if save_dir is None:
            plt.show()
        else:
            filename = f"{os.path.basename(path)[:5]}_{os.path.basename(path)[6:9]}"
            # filename = filename.lower()
            fig.savefig(f"{save_dir}/{filename}.png", 
                        bbox_inches="tight", 
                        dpi=72,
                        pad_inches=0.25, 
                        transparent=False)
            print(f"{i:02d} | {_session_name} -> saved.")
            
        plt.close()

    return fig


def calc_main_freq_and_amplitude_for_imu(signal, sec):
    '''
    # https://momonoki2017.blogspot.com/2018/03/pythonfft-2.html
    # https://momonoki2017.blogspot.com/2018/03/pythonfft-1-fft.html
    '''
    SAMPLING_RATE = 25
    WINDOW_LENGTH = sec
    N = SAMPLING_RATE*WINDOW_LENGTH
    t = np.arange(0, N, 1)
    dt = 1 / SAMPLING_RATE
    fq = np.linspace(0, 1.0/dt, N) 
    # print(fq)
    
    # https://ryo-iijima.com/fftresult/
    F = np.fft.fft(signal) # Results of Fast Fourier Transform (FFT)
    F_abs = np.abs(F) # Amplitude Spectrum (AS)

    # AS divided by (sampling rate/2) = nyquist frequency
    F_abs_amp = F_abs / SAMPLING_RATE * 2 
    # same as below
    # nyquist_frequency = int(SAMPLING_RATE/2)
    # F_abs_amp = F_abs / nyquist_frequency

    # amplitude at frequency 0 should be 0 
    # if there is direct current component, the value will be large. # F_abs_amp[0] / 2 
    F_abs_amp[0] = 0 
    
    ps = F_abs_amp*F_abs_amp # Power Spectrum
    psd = ps / SAMPLING_RATE
    
    # print(F_abs_amp)
    # print(len(F_abs_amp))
    idx_1 = np.argsort(F_abs_amp[:int(N/2)])[::-1][0]
    idx_2 = np.argsort(F_abs_amp[:int(N/2)])[::-1][1]
    main_freq_1, main_freq_2 = fq[idx_1], fq[idx_2]
    main_amp_1, main_amp_2 = F_abs_amp[idx_1], F_abs_amp[idx_2]
    main_psd_1, main_psd_2 = psd[idx_1], psd[idx_2]
    # fig, ax = plt.subplots(1, 1, figsize=(8, 4))
    # ax = sns.lineplot(x=fq, y=F_abs_amp)
    # ax = sns.lineplot(x=fq[:int(N/2)], y=F_abs_amp[:int(N/2)])
    # plt.show()
    # plt.close()
    return main_freq_1, main_amp_1, main_psd_1, main_freq_2, main_amp_2, main_psd_2


def plot_spectrogram(
        fig, ax, session_name, audio_file_type, 
        z, sampling_rate, sec, n_overlap):
    freqs, times, Sxx = signal.spectrogram(
        z, 
        fs=sampling_rate, 
        window='hanning',
        nperseg=sampling_rate, 
        noverlap=n_overlap,
        detrend='constant',
        scaling='spectrum')
    Sxx[Sxx==0] = np.finfo(float).eps
    # fig, ax = plt.subplots(1, 1, figsize=(10,8))
    spectrogram = ax.pcolormesh(times, 
                                freqs, 
                                np.log10(Sxx), # intensity to DB?
                                vmin=-7, vmax=0,
                                # Sxx, # raw intensity
                                # vmin=0, vmax=1e0
                                shading="auto"
                                )
    # ax.pcolormesh(times, freqs, 10*np.log10(Sx), cmap='viridis')
    ax.set_ylabel('Frequency[Hz]')
    ax.set_xlabel('Time[s]')
    cbar = fig.colorbar(spectrogram, ax=ax, orientation = "vertical")
    cbar.set_label("Amplitude")
    ax.set_title(f"session_name: {session_name} | {audio_file_type}", pad=10)
    
    return fig, ax


def get_audio_file_type(df):
    audio_file_type = np.unique(df['audio_file_type'])[0]
    return audio_file_type


def plot_video_frame_masked_pixel_data(
    df, 
    y_colname="abs_diff_pixel_count_p", 
    title=None
):
    
    # idx
    speaker_turn_on_idx = get_speaker_turn_on_idx(df)
    if speaker_turn_on_idx == 0:
        return None, None
    slide = 0
    sampling_rate = 30
    plot_before_sec = 30
    plot_after_sec = 30 + 5
    idx_0 = slide + speaker_turn_on_idx - plot_before_sec * sampling_rate
    idx_1 = slide + speaker_turn_on_idx + plot_after_sec * sampling_rate

    # figure
    fig, ax = plt.subplots(1, 1, figsize=(20, 5))
    ax.plot(df[y_colname], color=palette2[0])
    ax.axvline(x=speaker_turn_on_idx, color="black", linestyle="-")

    # ticks and ticklabels
    if 'count_p' in y_colname:
        ax.set_yticks(np.arange(0, 2, 0.2))
        ax.set_ylim(-0.1, 1.1)
    
    x_interval_sec = 5
    SR = sampling_rate
    xticks = np.arange(idx_0, idx_1 + 2*SR, SR*x_interval_sec)
    xticklabels = np.arange(idx_0/SR, ((idx_1) / SR) + 2, 1*x_interval_sec).astype(int)
    xticklabels = xticklabels - int(speaker_turn_on_idx / SR)
    if 0 not in xticklabels:
        closest_to_zero_idx = np.argmin(np.abs(xticklabels))
        shift = -xticklabels[closest_to_zero_idx]
        xticks = xticks + shift * SR
        xticklabels = xticklabels + shift
    ax.set_xticks(xticks)
    ax.set_xticklabels(xticklabels)
    ax.set_xlim(idx_0 - 1*SR+1, idx_1+1*SR-1)
    
    # axis labels
    ax.set_xlabel("Time (s)", labelpad=10)
    if y_colname in ['absdiff_pixel_count_p', 'abs_diff_pixel_count_p']:
        y_label = "AD-MPR"
    elif y_colname in ['pixel_count_p']:
        y_label = "MPR" # Masked Pixel Ratio
    elif y_colname == "pixel_count":
        y_label = "MPC" # Masked Pixel Count
    elif y_colname == "abs_diff_pixel_count":
        y_label = "AD-MPC"
    ax.set_ylabel(y_label, labelpad=10)

    # title
    ax.set_title(title, pad=20)

    # grid
    ax.xaxis.set_minor_locator(tck.AutoMinorLocator(x_interval_sec))
    ax.grid(which='major', alpha=0.40)
    ax.grid(which='minor', alpha=0.20)
    
    if speaker_turn_on_idx > 0:
        ax.axvspan(speaker_turn_on_idx, speaker_turn_on_idx+4.2*30, color="#808080", alpha=0.25)

    # plt.show()

    return fig, ax


def vis_masked_pixel_data(
    input_path_list,
    y_colname = "abs_diff_pixel_count_p",
    save_dir = None,
):

    for i, input_path in enumerate(input_path_list):
        # print(input_path)
        _session_name = os.path.basename(input_path)
        session_name = f"{os.path.basename(input_path)[:5]} | {os.path.basename(input_path)[6:9]}"
        title = f"{session_name}"
        df = pd.read_csv(input_path)
        # print(df.columns)
        idx = get_speaker_turn_on_idx(df)
        if idx == 0:
            print(f"{i:02d} | {_session_name} -> skipped.")
            continue
        
        # plot
        fig, ax = plot_video_frame_masked_pixel_data(
            df, y_colname = y_colname, title=title
        )

        # save
        if save_dir is None:
            plt.show()
        else:
            filename = f"{os.path.basename(input_path)[:5]}_{os.path.basename(input_path)[6:9]}"
            # filename = filename.lower()
            os.makedirs(save_dir, exist_ok=True)
            fig.savefig(f"{save_dir}/{filename}.png", 
                        bbox_inches="tight", 
                        dpi=72,
                        pad_inches=0.25, 
                        transparent=False)
            print(f"{i:02d} | {_session_name} -> saved.")
        plt.close()

# Spectrogram
def plot_audio_spectrogram(signal, sampling_rate, fig_title=None, cmap=None):
    # fft frame size
    fft_size = 2**8 # 256
    # fft_size = 2**10 # 1024

    # frame shift length
    hop_length = int(fft_size / 4)  
    # Short-time Foulier Transformation
    amplitude = np.abs(
        librosa.core.stft(   
            signal, 
            n_fft=fft_size, 
            hop_length=hop_length
        )
    )
    # Amplitude to db
    log_power = librosa.core.amplitude_to_db(amplitude, ref=np.max)
    
    # plot
    fig, ax = plt.subplots(1, 1, figsize=(14, 6))
    if cmap is None:
        color_map = 'magma' # 'magma', 'viridis', 'jet'
    else:
        color_map = cmap
    librosa.display.specshow(
        log_power, 
        sr=sampling_rate, 
        hop_length=hop_length, 
        x_axis='time', 
        y_axis='hz', 
        cmap=color_map
        )
    ax.set_xticks(np.arange(0, 100, 1))
    ax.set_xlim(0,30)
    plt.colorbar(format='%+2.0f dB')  
    if fig_title is not None:
        plt.title(fig_title, fontsize=16)
    ax.set_xlabel("Elapsed Time [s]", fontsize=14, labelpad=10)
    ax.set_ylabel("Frequency [Hz]", fontsize=14, labelpad=10)
    plt.tight_layout()
    plt.show()
    plt.close()

    return fig
    # fig.savefig(os.path.join(fig_save_dir, file_name)+"_spectrogram.png", dpi=300)


def load_logdata_and_prep_df_fix(path):
    ACC_SAMPLING_RATE = 25
    df = pd.read_csv(path)
    # display(df.head(5))
    # print(df.columns)

    print(f"len(df): {len(df)}")
    print(f"len(df): {len(df)/ACC_SAMPLING_RATE/60/60:.3f} h of logging data")

    # UTC, JST, unixtime
    df_time = rtc_data_time_to_timestamp_and_unixtime(df)
    # display(df_time.head(3))
    # display(df_time.tail(3))

    # GPS data
    df_gps = df_time[ (df_time["rtc_msec"] == 0) | (df_time["fix_type"] == 2) | (df_time["fix_type"] == 3)] # msec -> sec level
    # print(f"len(df_gps): {len(df_gps)}")

    df_fix = df_gps[ (df_gps["fix_type"] == 2) | (df_gps["fix_type"] == 3) ]
    # print(f"len(df_fix): {len(df_fix)}")
    print(f"{len(df_fix)/60:.1f} (m) of fixed GPS data")

    return df_fix


def insert_empty_rows(df, n_empty_rows):
    # create a new dataframe 
    new_rows = []
    for i in range(len(df)):
        new_rows.append(df.iloc[i])
        # add 29 empty rows
        for _ in range(n_empty_rows):
            new_rows.append(pd.Series({col: pd.NaT if col == 'datetime_jst' else None for col in df.columns}))
    new_df = pd.DataFrame(new_rows).reset_index(drop=True)
    return new_df



def calc_and_report_performance_metrics(y_true, y_pred):
    average_type_list = ["macro", "weighted", None]

    for average_type in average_type_list:
        print(f"--------------- {average_type} ----------------")
        # evaluation metrics
        accuracy = jaccard_score(y_true, y_pred, average=average_type)
        precision = precision_score(y_true, y_pred, average=average_type)
        recall = recall_score(y_true, y_pred, average=average_type)
        f1 = f1_score(y_true, y_pred, average=average_type)

        # print results
        if average_type is None:
            class_id_list = [0, 1]
            for class_id in class_id_list:
                print(f'class_id = {class_id}')
                print(f'Accuracy (Jaccard Score): {accuracy[class_id]:.2f}')
                print(f'Precision: {precision[class_id]:.2f}')
                print(f'Recall: {recall[class_id]:.2f}')
                print(f'F1 Score: {f1[class_id]:.2f}')
        else:
            print(f'Accuracy (Jaccard Score): {accuracy:.2f}')
            print(f'Precision: {precision:.2f}')
            print(f'Recall: {recall:.2f}')
            print(f'F1 Score: {f1:.2f}')

def plot_confusion_matrix_ax(ax, cm, labels=['Flying', 'Others']):

    # plot heatmap
    sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', xticklabels=labels, yticklabels=labels, cbar=True, ax=ax)
    
    # Remove the original colorbar
    ax.collections[0].colorbar.remove()
    
    # Normalize the colorbar to [0, 1]
    norm_cm = cm / cm.max()
    sm = plt.cm.ScalarMappable(cmap='Blues', norm=plt.Normalize(vmin=0, vmax=1))
    sm.set_array([])

    # Add the normalized colorbar
    cbar = ax.figure.colorbar(sm, ax=ax)
    cbar.set_ticks([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
    cbar.set_ticklabels(['0.0', '0.2', '0.4', '0.6', '0.8', '1.0'])
    
    # Remove the colorbar's frame
    cbar.outline.set_visible(False)

    ax.set_xlabel('Prediction', labelpad=20)
    ax.set_ylabel('Ground Truth', labelpad=20)

    return ax


def custom_formatter_2f(x, pos):
    x = np.round(x, 3)
    if x == 0:
        return '0.00'
    else:
        # print(x)
        return '{:.2f}'.format(x)
    
def custom_formatter_3f(x, pos):
    x = np.round(x, 4)
    if x == 0:
        return '0.000'
    else:
        return '{:.3f}'.format(x)
    