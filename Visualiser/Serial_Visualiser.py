import dash
import dash_core_components as dcc
import dash_html_components as html
import dash_bootstrap_components as dbc
from dash.dependencies import Input, Output
import plotly.graph_objs as go
import pandas as pd
import serial
import time
import os

# Initialise Dash app with a Bootstrap theme
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.CYBORG])

# Serial Configuration
SERIAL_PORT = '/dev/ttyUSB1' 
BAUD_RATE = 115200

CSV_FILE = "helmet_data.csv"

# Initialise Serial Connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) 
    print("Serial connection established.")
except Exception as e:
    print(f"Serial connection failed: {e}")
    ser = None

# Initialise empty DataFrame for live data
data = pd.DataFrame({"Time": [], "Yaw": [], "Pitch": [], "Roll": [], "Motion": [], "Lat": [], "Lng": [], "PIR_L": [], "PIR_R": []})

# Placeholder values for map and PIRs
latest_lat = 0.00
latest_lng = 0.00
latest_pir_l = 0
latest_pir_r = 0

if not os.path.isfile(CSV_FILE):
    data.to_csv(CSV_FILE, index=False)
    print(f"Created CSV file: {CSV_FILE}")


# Serial Data Reading Function
def read_serial_data():
    global latest_lat, latest_lng, latest_pir_l, latest_pir_r

    if ser and ser.isOpen():
        try:
            raw_data = ser.readline().decode('utf-8').strip()
            print(f"📥 Serial Data: {raw_data}")
            values = raw_data.split(',')
            if len(values) == 8:
                yaw = float(values[0])
                pitch = float(values[1])
                roll = float(values[2])
                motion = str(values[3])
                lat = float(values[4])
                lng = float(values[5])
                pir_l = bool(int(values[6]))
                pir_r = bool(int(values[7]))
                current_time = pd.Timestamp.now()

                latest_lat, latest_lng, latest_pir_l, latest_pir_r = lat, lng, pir_l, pir_r

                # Save to CSV
                new_entry = {
                    "Time": current_time, "Yaw": yaw, "Pitch": pitch, "Roll": roll,
                    "Motion": motion, "Lat": lat, "Lng": lng, "PIR_L": pir_l, "PIR_R": pir_r
                }
                pd.DataFrame([new_entry]).to_csv(CSV_FILE, mode='a', header=not os.path.isfile(CSV_FILE), index=False)
                print("Data saved to CSV.")

                return new_entry
        except Exception as e:
            print(f"Error reading serial data: {e}")
    return None


# Layout
app.layout = dbc.Container(
    fluid=True,
    children=[
        html.H1("Helmet Dashboard", style={"textAlign": "center", "marginTop": "20px", "color": "white"}),

        # Row 1: Google Map and Sensors
        dbc.Row(
            [
                # Google Map
                dbc.Col(
                    html.Iframe(
                        id="map-iframe",
                        src=f"https://www.google.com/maps?q={latest_lat},{latest_lng}&z=14&output=embed",
                        style={"width": "100%", "height": "400px", "border": "none", "borderRadius": "8px"},
                    ),
                    width=8,
                ),

                # PIR Indicators
                dbc.Col(
                    [
                        html.Div(
                            [
                                html.H4("Left PIR", style={"textAlign": "center", "color": "white"}),
                                html.Div(
                                    id="pir-left",
                                    style={
                                        "width": "100px",
                                        "height": "100px",
                                        "borderRadius": "50%",
                                        "border": "3px solid red",
                                        "margin": "auto",
                                    }
                                ),
                            ],
                            style={"marginBottom": "20px"},
                        ),
                        html.Div(
                            [
                                html.H4("Right PIR", style={"textAlign": "center", "color": "white"}),
                                html.Div(
                                    id="pir-right",
                                    style={
                                        "width": "100px",
                                        "height": "100px",
                                        "borderRadius": "50%",
                                        "border": "3px solid red",
                                        "margin": "auto",
                                    }
                                ),
                            ]
                        ),
                    ],
                    width=4,
                    style={"textAlign": "center"},
                ),
            ],
            style={"marginBottom": "30px"},
        ),

        # Row 2: Live Graph and 'No turn detected' box
        dbc.Row(
            [
                dbc.Col(
                    dcc.Graph(id="live-graph", style={"height": "400px"}),
                    width=10,
                ),
                dbc.Col(
                    dbc.Card(
                        dbc.CardBody(
                            html.H4("No turn detected", style={"textAlign": "center", "color": "white"})
                        ),
                        style={"backgroundColor": "#333333", "color": "white", "borderRadius": "8px"},
                    ),
                    width=2,
                ),
            ],
            style={"marginBottom": "30px"},
        ),

        # Interval for updates
        dcc.Interval(
            id="interval-update",
            interval=300,
            n_intervals=0,
        ),
    ],
)


# Update the live graph
@app.callback(
    [Output("live-graph", "figure"),
     Output("pir-left", "style"),
     Output("pir-right", "style"),
     Output("map-iframe", "src")],
    [Input("interval-update", "n_intervals")]
)
def update_dashboard(n_intervals):
    global data, latest_lat, latest_lng, latest_pir_l, latest_pir_r

    # Read new data from Serial
    new_row = read_serial_data()
    if new_row:
        new_data = pd.DataFrame([new_row])
        data = pd.concat([data, new_data]).tail(50)

    pir_left_style = {"backgroundColor": "red" if latest_pir_l else "transparent"}
    pir_right_style = {"backgroundColor": "red" if latest_pir_r else "transparent"}

    map_src = f"https://www.google.com/maps?q={latest_lat},{latest_lng}&z=14&output=embed"

    figure = {"data": [go.Scatter(x=data["Time"], y=data["Yaw"], mode="lines")]}

    return figure, pir_left_style, pir_right_style, map_src


if __name__ == "__main__":
    try:
        app.run_server(debug=True)
    except KeyboardInterrupt:
        if ser:
            ser.close()
        print("Serial connection closed.")
