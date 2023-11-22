import csv

def load_location_data(file_path):
    locations_data = []
    with open(file_path, 'r') as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=',')
        for row in csv_reader:
            location_name = row[0].strip()
            origin_lat = float(row[1].strip()) 
            origin_lon = float(row[2].strip())
            csv_file_path = row[3].strip()
            locations_data.append({
                'location_name': location_name,
                'origin_lat': origin_lat,
                'origin_lon': origin_lon,
                'csv_file_path': csv_file_path
            })
    return locations_data
