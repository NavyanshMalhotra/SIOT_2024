import dash
import dash_core_components as dcc
import dash_html_components as html
import dash_bootstrap_components as dbc
from dash.dependencies import Input, Output
import plotly.graph_objs as go
import pandas as pd
import serial
import time

app = dash.Dash(__name__, external_stylesheets=[dbc.themes.CYBORG])

SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 115200

# Initialize Serial Connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) 
    print("Serial connection established.")
except Exception as e:
    print(f"Serial connection failed: {e}")
    ser = None

data = pd.DataFrame({"Time": [], "Yaw": [], "Pitch": [], "Roll": [], "Lat": [], "Lng": []})

def read_serial_data():
    if ser and ser.isOpen():
        try:
            raw_data = ser.readline().decode('utf-8').strip()
            print(f"📥 Serial Data: {raw_data}")
            values = raw_data.split(',')
            if len(values) == 5:  
                yaw = float(values[0])
                pitch = float(values[1])
                roll = float(values[2])
                lat = float(values[3])
                lng = float(values[4])
                current_time = pd.Timestamp.now() - pd.Timedelta(hours=7)
                return {"Time": current_time, "Yaw": yaw, "Pitch": pitch, "Roll": roll, "Lat": lat, "Lng": lng}
        except Exception as e:
            print(f"Error reading serial data: {e}")
    return None

# Layout
app.layout = dbc.Container(
    fluid=True,
    children=[
        html.H1("Helmet Dashboard", style={"textAlign": "center", "marginTop": "20px", "color": "white"}),

        dbc.Row(
            [
                # Google Map
                dbc.Col(
                    html.Iframe(
                        src="https://www.google.com/maps?q=" + str(lat) + "," + str(lng) + "&z=14&output=embed",
                        style={"width": "100%", "height": "400px", "border": "none", "borderRadius": "8px"},
                    ),
                    width=8,
                ),

                # Round Sensors
                dbc.Col(
                    [
                        html.Div(
                            children=[
                                html.H4("Left PIR", style={"textAlign": "center", "color": "white"}),
                                html.Div(
                                    style={
                                        "width": "100px",
                                        "height": "100px",
                                        "borderRadius": "50%",
                                        "backgroundColor": "red",
                                        "margin": "auto",
                                    }
                                ),
                            ],
                            style={"marginBottom": "20px"},
                        ),
                        html.Div(
                            children=[
                                html.H4("Right PIR", style={"textAlign": "center", "color": "white"}),
                                html.Div(
                                    style={
                                        "width": "100px",
                                        "height": "100px",
                                        "borderRadius": "50%",
                                        "backgroundColor": "red",
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

        # Interval component for live updates
        dcc.Interval(
            id="interval-update",
            interval= 300,  # Update every 300 ms
            n_intervals=0,
        ),
    ],
)

# Update the live graph
@app.callback(
    Output("live-graph", "figure"),
    [Input("interval-update", "n_intervals")]
)
def update_graph(n_intervals):
    global data

    # Read new data from Serial
    new_row = read_serial_data()
    if new_row:
        new_data = pd.DataFrame([new_row])
        data = pd.concat([data, new_data]).tail(50)  # Keep the last 50 rows

    # Create updated figure
    figure = {
        "data": [
            go.Scatter(x=data["Time"], y=data["Yaw"], mode="lines", name="Yaw"),
            go.Scatter(x=data["Time"], y=data["Pitch"], mode="lines", name="Pitch"),
            go.Scatter(x=data["Time"], y=data["Roll"], mode="lines", name="Roll"),
        ],
        "layout": go.Layout(
            title="Yaw, Pitch, and Roll Live Data",
            xaxis={"title": "Time"},
            yaxis={"title": "Degrees"},
            paper_bgcolor="#222222",
            plot_bgcolor="#222222",
            font={"color": "white"},
        ),
    }
    return figure

# Run the app
if __name__ == "__main__":
    try:
        app.run_server(debug=True)
    except KeyboardInterrupt:
        if ser:
            ser.close()
        print("Serial connection closed.")
