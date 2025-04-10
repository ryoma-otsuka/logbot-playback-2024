import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import folium
from folium.plugins import TimestampedGeoJson
import geopandas as gpd
from shapely.geometry import Point, LineString
from pyproj import Transformer
import contextily as ctx


"""

GPS Trajetory Map (.html)

"""

def plot_trajectory_on_map_and_save_as_html(
    df, 
    test_dir, 
    test_id, 
    only_cam_recording_session=False
):

    # Color set: [GPS trajectory, video_recording, playback]
    # COLOR_LIST = ['blue', 'red', 'yellow']
    # COLOR_LIST = ['#56B4E9', '#E69F00', '#009E73'] # C1
    # COLOR_LIST = ['#00bbdf', '#ff4554', '#bad600'] # C2
    COLOR_LIST = ["#1E88E5", "#D81B60", "#FFC107"] # C3

    kabushima_shrine = [40.538570, 141.557596] 
    _location = kabushima_shrine

    m = folium.Map(
        location=_location,
        tiles='OpenStreetMap',
        # tiles='Cartodb Positron',
        # tiles='Stamen Terrain',
        zoom_start=15,
        control_scale=True,
        position='relative'
        # position='absolute'
    )

    # Kabushima Bounding Box
    NE = [40.5477, 141.5694]
    NW = [40.5477, 141.5456]
    SW = [40.5294, 141.5456]
    SE = [40.5294, 141.5694]

    # plot Umineko Black box
    folium.Polygon(
        locations=[NE, NW, SW, SE],
        color="#BBBBBB",
        # popup='Black box: Kabushima + 1km',
        weight=3,
        fill=True,
        fill_opacity=0.3
    ).add_to(m)

    features = []
    pseudo_start_time = pd.Timestamp('2023-01-01 00:00:00')
    pseudo_time = pseudo_start_time

    for index, item in df.iterrows():

        camera_count = item['camera_count']
        camera_recording = item['camera_recording']
        
        # if only_cam_recording_session == True:
        #     if camera_recording == 0:
        #         continue # skip

        fix_type = item['fix_type']
        if fix_type == 3:
            COLOR_0 = COLOR_LIST[0]
        elif fix_type == 2:
            COLOR_0 = "#BBBBBB"

        
        lat_lon_tmp = [item['latitude'], item['longitude']]
        folium.Circle(
            location=lat_lon_tmp,
            radius=1,
            color=COLOR_0,
            fill=True, 
            fill_color=COLOR_0, 
            popup=f"{item['datetime_jst'], item['fix_type'], item['camera_count']}"
        ).add_to(m)
        
        camera_recording = item['camera_recording']
        COLOR_1 = COLOR_LIST[1]
        
        # Exception handler (RTC timestamp error)
        if test_id == "LBP03" and item['rtc_hour'] == 19 and item['rtc_min'] == 0 and item['rtc_sec'] == 0:
            continue # skip

        if camera_recording == 1:
            lat_lon_tmp = [item['latitude'], item['longitude']]
            folium.Circle(
                location=lat_lon_tmp,
                radius=3,
                color=COLOR_1,
                fill=True, 
                fill_color=COLOR_1,
                popup=f"{item['datetime_jst'], item['fix_type'], item['camera_count']}"
            ).add_to(m)

        speaker_on = item['speaker_on']
        if speaker_on == 1:
            lat_lon_tmp = [item['latitude'], item['longitude']]
            COLOR_2 = COLOR_LIST[2]
            folium.Circle(
            # folium.CircleMarker(
                location=lat_lon_tmp,
                radius=3,
                color=COLOR_2,
                fill=True, 
                fill_color=COLOR_2,
                popup=f"{item['datetime_jst'], item['fix_type'], item['audio_file']}"
            ).add_to(m)
        
        # Skip if the bird was inside Kabushima Bounding Box
        if camera_recording == 0 and item['latitude'] < NE[0] and item['latitude'] > SW[0] and item['longitude'] < NE[1] and item['longitude'] > SW[1]:
            continue
        
        if only_cam_recording_session == True:
            if camera_recording == 0:
                continue

        # COLOR_4 = "#FFC107"
        COLOR_4 = "white"
        # COLOR_5 = "Black"
        COLOR_5 = "#333333"
        feature = {
            'type': 'Feature',
            'geometry': {
                'type': 'Point',
                'coordinates': [item['longitude'], item['latitude']]
            },
            'properties': {
                # 'time': item['datetime_jst'].isoformat(),
                'time': pseudo_time.isoformat(),
                'icon': 'circle',
                'iconstyle': {
                    'color': COLOR_5,
                    'fillColor': COLOR_4,
                    'fillOpacity': 0.75,
                    'stroke': 'true',
                    'radius': 6
                },
                'popup': f"Time: {item['datetime_jst']}<br>Fix Type: {item['fix_type']}<br>Camera Recording: {item['camera_recording']}<br>Speaker On: {item['speaker_on']}"
            }
        }
        features.append(feature)
        pseudo_time += pd.Timedelta(seconds=1)  # increment by 1 sec
        
    geojson = {
        'type': 'FeatureCollection',
        'features': features
    }

    TimestampedGeoJson(
        geojson,
        period='PT1S',  # 1 sec update time (1 Hz)
        add_last_point=True,
        auto_play=False,
        loop=False,
        max_speed=8192, # 4, 8, 16, 64, 128, 256, 512, 1024, 2048, 4096, 8192
        loop_button=True,
        date_options='YYYY-MM-DD HH:mm:ss',
        time_slider_drag_update=True,
        duration='PT1S',  # 1 sec update time (1 Hz)
    ).add_to(m)

    if only_cam_recording_session == True:
        test_dir = test_dir + "-2"
    
    # if session_name is None:
    #     outfile_path = f"../map/{test_dir}/{test_id}.html"
    # else:
    #     outfile_path = f"../map/{test_dir}/{test_id}/{session_name}.html"
    outfile_path = f"../output/map/{test_dir}/{test_id}.html"
    print(outfile_path)
    outfile_dir = os.path.dirname(outfile_path)
    if os.path.exists(outfile_dir) == False:
        os.makedirs(outfile_dir)

    m.save(outfile=outfile_path)

    return m


"""

GPS Trajectory Map (.png, .svg)

"""

def generate_random_trajectory(
    random_type="random-walk",
    start_lat=40.538570,
    start_lon=141.557596,
    goal_lat=41.0,
    goal_lon=142.0,
    step_degree=0.005,
    random_seed=3,
    n_steps=1000
):
    np.random.seed(random_seed)
    num_points = n_steps
    lats = [start_lat]
    lons = [start_lon]
    goal = np.array([goal_lat, goal_lon])

    if random_type == "random-walk":
        for _ in range(n_steps):
            lats.append(lats[-1] + np.random.uniform(-step_degree, step_degree))
            lons.append(lons[-1] + np.random.uniform(-step_degree, step_degree))
    elif random_type == "levy-flights":
        alpha = 1.5  # a parameter of Pareto distribution
        for _ in range(n_steps):
            step_size = np.random.pareto(alpha) * step_degree
            angle = np.random.uniform(0, 2 * np.pi)
            lats.append(lats[-1] + step_size * np.cos(angle))
            lons.append(lons[-1] + step_size * np.sin(angle))
    else:
        raise Exception(f"Unknown {random_type}")
    # lats.append(goal_lat)
    # lons.append(goal_lon)

    return lats, lons


def convert_single_lat_lon_set_to_web_mercator(
    lat,
    lon
):
    gdf = gpd.GeoDataFrame(geometry=[Point(lon, lat)], crs="EPSG:4326")
    gdf = gdf.to_crs(epsg=3857)

    return gdf


def mercator_to_latlon(x, y):
    transformer = Transformer.from_crs(
        "EPSG:3857", 
        "EPSG:4326", 
        always_xy=True
    )
    return transformer.transform(x, y)

def latlon_to_mercator(x, y):
    transformer = Transformer.from_crs(
        "EPSG:4326", 
        "EPSG:3857", 
        always_xy=True
    )
    return transformer.transform(x, y)

# Plot trajectory map for paper & poster presentation
def vis_trajectory_map(
    lats, 
    lons,
    colors,
    base_loc=[40.538570, 141.557596],
    plot_line=False,
    plot_line_only=False,
    ax3_lats_lons_colors=None,
):
    # Base location
    base_gdf = convert_single_lat_lon_set_to_web_mercator(
        base_loc[0], base_loc[1]
    )

    # make GeoDataFrame
    geometry = [Point(lon, lat) for lon, lat in zip(lons, lats)]
    gdf = gpd.GeoDataFrame(geometry=geometry, crs="EPSG:4326")

    # make LineString (trajectory data)
    line = LineString(geometry)
    gdf_line = gpd.GeoDataFrame(geometry=[line], crs="EPSG:4326")

    # convert GeoDataFrame to Web Mercator
    gdf = gdf.to_crs(epsg=3857)
    gdf_line = gdf_line.to_crs(epsg=3857)
    # gdf = gdf.to_crs(epsg=4326)
    # gdf_line = gdf_line.to_crs(epsg=4326)

    # plot
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    minlon, maxlon = np.min(base_gdf.geometry.x), np.max(base_gdf.geometry.x)
    minlat, maxlat = np.min(base_gdf.geometry.y), np.max(base_gdf.geometry.y)
    ax.set_xlim(minlon - 36000, maxlon + 80000)  # margin for longitude
    ax.set_ylim(minlat - 18000, maxlat + 78000)  # margin for latitude

    MARKER_SIZE = 0.5
    LINE_WIDTH = 1.0
    ALPHA = 0.5
    
    # plot the converted Web Mercator data
    if plot_line_only == False:
        gdf.plot(
            ax=ax, 
            color=colors, 
            marker='o', 
            markersize=MARKER_SIZE, 
            alpha=ALPHA,
            label='GPS Points'
        )
    if plot_line == True:
        gdf_line.plot(
            ax=ax, 
            color=colors, 
            linewidth=LINE_WIDTH, 
            label='GPS Track'
        )

    # add base map
    providers = ctx.providers.flatten()
    providers_name = [
        'OpenStreetMap.Mapnik', # 0
        'OpenStreetMap.HOT',    # 1
        'OpenTopoMap',          # 2
        'CartoDB.Positron',     # 3 Simple & Fast
        'CartoDB.Voyager',      # 4
        'CartoDB.DarkMatter',   # 5
        'NASAGIBS.ASTER_GDEM_Greyscale_Shaded_Relief',
        'NASAGIBS.ViirsEarthAtNight2012',
        'NASAGIBS.BlueMarble',
        'USGS.USTopo'
    ]
    ctx.add_basemap(
        ax, 
        # crs=gdf.crs,
        crs=gdf.crs.to_string(), 
        # crs="EPSG:4326",
        source=providers[providers_name[3]],
        attribution=""
    )

    # axis labels
    x_labels = ax.get_xticks()
    y_labels = ax.get_yticks()

    # y ticks
    x_ticks_lon = [141.40, 141.60, 141.80, 142.00, 142.20]
    x_ticks_mercator = [latlon_to_mercator(lon, 0)[0] for lon in x_ticks_lon]
    y_ticks_lat = [40.50, 40.60, 40.70, 40.80, 40.90, 41.00]
    y_ticks_mercator = [latlon_to_mercator(0, lat)[1] for lat in y_ticks_lat]

    # x ticks
    ax.set_xticks(x_ticks_mercator)
    ax.set_xticklabels([f'{lon:.1f}' for lon in x_ticks_lon], fontsize=18)
    ax.set_yticks(y_ticks_mercator)
    ax.set_yticklabels([f'{lat:.1f}' for lat in y_ticks_lat], fontsize=18)

    # Japan map
    ax2 = ax.inset_axes([0.52, 0.02, 0.46, 0.5]) # lower right
    # extent = [1.2e6, 2.8e6, 3.2e6, 4.8e6]
    minlon, maxlon = np.min(base_gdf.geometry.x), np.max(base_gdf.geometry.x)
    minlat, maxlat = np.min(base_gdf.geometry.y), np.max(base_gdf.geometry.y)
    ax2.set_xlim(minlon - 2500000, maxlon + 1400000)  # margin for longitude
    ax2.set_ylim(minlat - 1800000, maxlat + 1600000)  # margin for latitude

    _base_gdf = base_gdf.to_crs(epsg=3857)
    _base_gdf.plot(
        ax=ax2, color="black", marker='*', 
        markersize=50, alpha=1.0, label='Kabushima'
    )
    ctx.add_basemap(
        ax2, crs=gdf.crs.to_string(), 
        source=providers[providers_name[3]], 
        attribution=""
    )
    ax2.set_xticks([])
    ax2.set_yticks([]) 
    BOX_LINE_WIDTH = 1.0
    BOX_LINE_COLOUR = 'black'
    box_line_list = ['top', 'bottom', 'left', 'right']
    for box_line in box_line_list:
        ax2.spines[box_line].set_linewidth(BOX_LINE_WIDTH)
        ax2.spines[box_line].set_color(BOX_LINE_COLOUR)

    if ax3_lats_lons_colors is not None:
        lons = ax3_lats_lons_colors['lons']
        lats = ax3_lats_lons_colors['lats']
        colors = ax3_lats_lons_colors['colors']

        # make GeoDataFrame
        geometry = [Point(lon, lat) for lon, lat in zip(lons, lats)]
        gdf = gpd.GeoDataFrame(geometry=geometry, crs="EPSG:4326")

        # make LineString (trajectory data)
        line = LineString(geometry)
        gdf_line = gpd.GeoDataFrame(geometry=[line], crs="EPSG:4326")

        # convert GeoDataFrame to Web Mercator
        gdf = gdf.to_crs(epsg=3857)
        gdf_line = gdf_line.to_crs(epsg=3857)

        # Enlarge the trajectory before and after the audio playback and
        # display it in the bottom-right space
        ax3 = ax.inset_axes([0.6, 0.36, 0.40, 0.40]) # lower right
        ax3.set_xticks([])
        ax3.set_yticks([]) 
        BOX_LINE_WIDTH = 1.5
        BOX_LINE_COLOUR = 'black'
        box_line_list = ['top', 'bottom', 'left', 'right']
        for box_line in box_line_list:
            ax3.spines[box_line].set_linewidth(BOX_LINE_WIDTH)
            ax3.spines[box_line].set_color(BOX_LINE_COLOUR)
        
        # Display the trajectory before and after the playback, 
        # centered on the trajectory during the intervention
        intervention_lat = ax3_lats_lons_colors['intervention_lat']
        intervention_lon = ax3_lats_lons_colors['intervention_lon']
        intervention_gdf = convert_single_lat_lon_set_to_web_mercator(
            intervention_lat, intervention_lon
        )
        _lat = np.median(intervention_gdf.geometry.x)
        _lon = np.median(intervention_gdf.geometry.y)
        ax3.set_xlim(_lat - 70, _lat + 70)
        ax3.set_ylim(_lon - 80, _lon + 80)

        MARKER_SIZE = 5.0
        LINE_WIDTH = 1.0
        ALPHA = 0.9
        
        # plot the converted Web Mercator data
        if plot_line_only == False:
            
            gdf.plot(
                ax=ax3, 
                color=colors, 
                marker='o', 
                markersize=MARKER_SIZE, 
                alpha=ALPHA,
                label='GPS Points'
            )
        if plot_line == True:
            gdf_line.plot(
                ax=ax3, 
                color=colors, 
                linewidth=LINE_WIDTH, 
                label='GPS Track'
            )

        ctx.add_basemap(
        ax3, 
        # crs=gdf.crs,
        crs=gdf.crs.to_string(), 
        # crs="EPSG:4326",
        source=providers[providers_name[3]],
        attribution=""
    )
    
    plt.tight_layout()
    plt.show()

    return fig