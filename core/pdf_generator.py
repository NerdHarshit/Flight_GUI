from fpdf import FPDF
import webbrowser
import datetime
from gui.plots import LivePlot

class PDFReport:
    @staticmethod

    def generate(buffer, acc_plot,height_plot):

        acc_plot.save_plot("acc_plot.png")
        height_plot.save_plot("height_plot.png")

        packets = buffer.data #assumiing list of packets

        if not packets :
            print("No data for report")
            return
        
        #timestamps part
        start_time = packets[0]["timestamp"] / 1000
        end_time = packets[-1]["timestamp"]/1000
        total_time = round(end_time - start_time ,2)

        #fsm events
        liftoff = apogee = parachute = landed = None

        for p in packets:
            state = p["FSM"]
            t = p['timestamp'] /1000

            if state ==1 and liftoff is None:
                liftoff = t
            
            if state ==4 and apogee is None:
                apogee = t

            if state ==5 and parachute is None:
                parachute = t

            if state ==7:
                landed = t
            
        #max values logic here
        max_ax = max(p['Ax'] for p in packets)
        max_ay = max(p['Ay'] for p in packets)
        max_az = max(p['Az'] for p in packets)
        max_h_baro = max(p["H_baro"] for p in packets)
        max_h_gps = max(p["H_gps"] for p in packets)

        #add v logic later using calculations engine

        #location logic
        launch_pkt = packets[0]
        apogee_pkt = max(packets, key = lambda p: p["H_baro"])
        landing_pkt = next((p for p in packets if p["FSM"] == 7), packets[-1])

        def maps_links(lat,lon):
            google_link = f"https://www.google.com/maps?q={lat},{lon}"
            return google_link
        
        #-------pdf creation starts here -------

        pdf = FPDF()
        pdf.set_auto_page_break(auto=True, margin =10)

        #----page 1 logic----
        pdf.add_page()

        try :
            pdf.image("Assets\\impulse_logo_black.png",x = 80,w=50)
            #pdf.image("Assets\\svkm_logo.png",x=10,w=30)
        
        except:
            pass
        
        pdf.ln(10)

        now = datetime.datetime.now()
        title = f"Flight Report : FR_{now.strftime('%d_%m_%Y_%H_%M')}"
        pdf.set_font("Arial","B",16)
        pdf.cell(0,10,title,ln=True)

        pdf.set_font("Arial","",12)
        pdf.cell(0, 8, f"Flight conducted on: {now.strftime('%A, %d %B %Y')}", ln=True)

        pdf.ln(5)

        #important timestamps section
        pdf.set_font("Arial", "B", 14)
        pdf.cell(0,10,"Important Timestamps: ",ln=True)

        pdf.set_font("Arial", "B", 11)

        pdf.cell(0, 6, f"Liftoff: {liftoff}", ln=True)
        pdf.cell(0, 6, f"Max Acceleration at: {liftoff}", ln=True)
        pdf.cell(0, 6, f"Apogee at: {apogee}", ln=True)
        pdf.cell(0, 6, f"Parachute deployed at: {parachute}", ln=True)
        pdf.cell(0, 6, f"Landed at: {landed}", ln=True)
        pdf.cell(0, 6, f"Total flight time: {total_time}", ln=True)

        pdf.ln(5)

        pdf.set_font("Arial", "B", 14)
        pdf.cell(0,10,"Important Locations: ",ln=True)

        pdf.set_font("Arial", "B", 11)

        def add_location(name,pkt):
            lat , lon = pkt["Latitude"] , pkt["Longitude"]
            link = maps_links(lat,lon)

            pdf.cell(0,6,f"{name} : {lat} , {lon}",ln=True,link=link)

        add_location("Launch_site",launch_pkt)
        add_location("Apogee_site",apogee_pkt)
        add_location("Landing site",landing_pkt)

        pdf.ln(5)

        pdf.set_font("Arial", "B", 14)
        pdf.cell(0,10,"Important Telemetry: ",ln=True)
        pdf.set_font("Arial", "B", 11)

        pdf.cell(0, 6, f"Max Ax: {round(max_ax,2)}", ln=True)
        pdf.cell(0, 6, f"Max Ay: {round(max_ay,2)}", ln=True)
        pdf.cell(0, 6, f"Max Az: {round(max_az,2)}", ln=True)

        pdf.cell(0, 6, f"Apogee Barometer Height: {round(max_h_baro,2)}", ln=True)
        pdf.cell(0, 6, f"Apogee GPS Height: {round(max_h_gps,2)}", ln=True)

        pdf.ln(5)

        pdf.image("acc_plot.png",w=90)
        pdf.image("height_plot.png",w=90)

        #page 2 starts here
        pdf.add_page()

        pdf.set_font("Arial", "B", 14)
        pdf.cell(0, 10, "Flight States Achieved:", ln=True)

        states = sorted(set(p["FSM"] for p in packets))
        pdf.set_font("Arial", "", 11)
        pdf.cell(0, 6, f"States: {states}", ln=True)

        lost = buffer.get_packet_loss()
        pdf.cell(0, 6, f"Total Packets Lost: {lost}", ln=True)

        pdf.ln(10)

        pdf.cell(0,10,"------ END OF REPORT ------",ln=True,align='C')

        pdf.ln(10)

        pdf.set_font("Arial", "", 14)

        pdf.cell(0,6,"DJS Impulse",ln=True)

        pdf.cell(0, 6, "Instagram", link="https://www.instagram.com/djs_impulse/", ln=True,)
        pdf.cell(0, 6, "LinkedIn", link="https://www.linkedin.com/company/djs-impulse/", ln=True)
        pdf.cell(0, 6, "YouTube", link="http://www.youtube.com/@DJSImpulse", ln=True)

        pdf.ln(5)

        pdf.cell(0, 6, "Designed and coded by Harshit Pandya", align="R",link="https://github.com/NerdHarshit")

        pdf.output("flight_report.pdf")

        print("PDF Generated!")
        
