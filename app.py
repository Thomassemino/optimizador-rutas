from flask import Flask, render_template, request, jsonify
import requests
import json
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import folium
from typing import List, Tuple, Dict
import time
import math

app = Flask(__name__)

# ========================================
# 🔑 CONFIGURACIÓN - COLOCA TU API KEY AQUÍ
# ========================================
OPENROUTESERVICE_API_KEY = "eyJvcmciOiI1YjNjZTM1OTc4NTExMTAwMDFjZjYyNDgiLCJpZCI6IjVkOTM5MDc5ZjE3YTRkMTJhMzMyZmY4YTdkN2RlMDhlIiwiaCI6Im11cm11cjY0In0="

# Para obtener tu API key gratis:
# 1. Ve a https://openrouteservice.org/dev/#/signup
# 2. Registrate y confirma tu email
# 3. Ve a tu dashboard y copia la API key
# 4. Reemplaza "TU_API_KEY_AQUI" con tu key real

class RouteOptimizer:
    def __init__(self, api_key: str):
        self.api_key = api_key
        self.base_url = "https://api.openrouteservice.org"
        
    def normalize_address(self, address: str) -> str:
        """Normaliza direcciones para mejorar geocodificación"""
        address = address.strip()
        
        replacements = {
            # CABA (Ciudad Autónoma de Buenos Aires)
            ', caba': ', Ciudad Autónoma de Buenos Aires, Argentina',
            ', CABA': ', Ciudad Autónoma de Buenos Aires, Argentina', 
            ', Capital Federal': ', Ciudad Autónoma de Buenos Aires, Argentina',
            ', CF': ', Ciudad Autónoma de Buenos Aires, Argentina',
            ', cap fed': ', Ciudad Autónoma de Buenos Aires, Argentina',
            ', capital': ', Ciudad Autónoma de Buenos Aires, Argentina',
            
            # Provincia de Buenos Aires
            ', pba': ', Buenos Aires, Argentina',
            ', PBA': ', Buenos Aires, Argentina',
            ', bs as': ', Buenos Aires, Argentina',
            
            # Abreviaturas comunes
            'Bme ': 'Bartolomé ',
            'Bme.': 'Bartolomé',
            'Av ': 'Avenida ',
            'Gral ': 'General ',
            'Pte ': 'Presidente ',
            'Dr ': 'Doctor ',
            'Ing ': 'Ingeniero ',
            'Cnel ': 'Coronel ',
            'Alte ': 'Almirante '
        }
        
        for old, new in replacements.items():
            address = address.replace(old, new)
        
        # Agregar contexto geográfico si falta
        if not any(x in address.lower() for x in ['argentina', 'buenos aires', 'caba', 'ciudad autónoma']):
            if 'lobos' in address.lower():
                address += ', Buenos Aires, Argentina'
            else:
                address += ', Ciudad Autónoma de Buenos Aires, Argentina'
        
        return address
    
    def geocode_address(self, address: str) -> Tuple[float, float]:
        """Convierte dirección en coordenadas lat,lng"""
        normalized_address = self.normalize_address(address)
        
        url = f"{self.base_url}/geocode/search"
        params = {
            'api_key': self.api_key,
            'text': normalized_address,
            'size': 3,
            'boundary.country': 'AR'
        }
        
        response = requests.get(url, params=params, timeout=15)
        response.raise_for_status()
        data = response.json()
        
        if data['features']:
            # Buscar resultado en Argentina
            for feature in data['features']:
                coords = feature['geometry']['coordinates']
                lat, lng = coords[1], coords[0]
                
                if -55 <= lat <= -21 and -74 <= lng <= -53:
                    return (lat, lng)
            
            # Si no encuentra en Argentina, usar primer resultado
            coords = data['features'][0]['geometry']['coordinates']
            return (coords[1], coords[0])
        
        # Reintentar con dirección original
        if normalized_address != address:
            params['text'] = address
            response = requests.get(url, params=params, timeout=15)
            response.raise_for_status()
            data = response.json()
            
            if data['features']:
                coords = data['features'][0]['geometry']['coordinates']
                return (coords[1], coords[0])
        
        raise ValueError(f"No se encontró la dirección: {address}")
    
    def haversine_distance(self, coord1: Tuple[float, float], coord2: Tuple[float, float]) -> float:
        """Calcula distancia haversine entre dos puntos en km"""
        if coord1 is None or coord2 is None or len(coord1) != 2 or len(coord2) != 2:
            raise ValueError("Coordenadas inválidas")
        
        lat1, lon1 = coord1
        lat2, lon2 = coord2
        
        for val in [lat1, lon1, lat2, lon2]:
            if val is None or not isinstance(val, (int, float)):
                raise ValueError(f"Coordenada inválida: {val}")
        
        R = 6371  # Radio de la Tierra en km
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = (math.sin(dlat/2)**2 + 
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * 
             math.sin(dlon/2)**2)
        
        c = 2 * math.asin(math.sqrt(a))
        return R * c
    
    def create_distance_matrix(self, coordinates: List[Tuple[float, float]]) -> List[List[float]]:
        """Crea matriz de distancias usando cálculo haversine"""
        # Validar coordenadas
        for i, coord in enumerate(coordinates):
            if coord is None or len(coord) != 2 or coord[0] is None or coord[1] is None:
                raise ValueError(f"Coordenada {i} inválida: {coord}")
        
        n = len(coordinates)
        matrix = [[0.0 for _ in range(n)] for _ in range(n)]
        
        for i in range(n):
            for j in range(n):
                if i != j:
                    matrix[i][j] = self.haversine_distance(coordinates[i], coordinates[j])
        
        return matrix
    
    def solve_tsp(self, distance_matrix: List[List[float]], start_index: int = 0) -> List[int]:
        """Resuelve TSP usando OR-Tools"""
        manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, start_index)
        routing = pywrapcp.RoutingModel(manager)
        
        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return int(distance_matrix[from_node][to_node] * 1000)
        
        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.seconds = 30
        
        solution = routing.SolveWithParameters(search_parameters)
        
        if solution:
            route = []
            index = routing.Start(0)
            while not routing.IsEnd(index):
                route.append(manager.IndexToNode(index))
                index = solution.Value(routing.NextVar(index))
            return route
        else:
            raise ValueError("No se pudo encontrar una solución óptima")
    
    def create_google_maps_link(self, addresses: List[str], optimized_route: List[int]) -> str:
        """Crea link de Google Maps con waypoints"""
        ordered_addresses = [addresses[i] for i in optimized_route]
        
        if len(ordered_addresses) == 1:
            origin = ordered_addresses[0].replace(' ', '+')
            return f"https://www.google.com/maps/search/{origin}"
        
        origin = ordered_addresses[0].replace(' ', '+')
        destination = ordered_addresses[-1].replace(' ', '+')
        
        waypoints = [addr.replace(' ', '+') for addr in ordered_addresses[1:-1]]
        
        # Limitar waypoints (Google Maps free = 8 waypoints máx)
        if len(waypoints) > 8:
            waypoints = waypoints[:8]
        
        url = f"https://www.google.com/maps/dir/{origin}"
        for waypoint in waypoints:
            url += f"/{waypoint}"
        url += f"/{destination}"
        
        return url
    
    def create_map(self, coordinates: List[Tuple[float, float]], 
                   addresses: List[str], optimized_route: List[int]) -> str:
        """Crea mapa interactivo con Folium"""
        center_lat = sum(coord[0] for coord in coordinates) / len(coordinates)
        center_lng = sum(coord[1] for coord in coordinates) / len(coordinates)
        
        m = folium.Map(location=[center_lat, center_lng], zoom_start=11)
        
        colors = ['red', 'blue', 'green', 'purple', 'orange', 'darkred', 
                 'lightred', 'beige', 'darkblue', 'darkgreen', 'cadetblue']
        
        for i, route_index in enumerate(optimized_route):
            coord = coordinates[route_index]
            color = colors[i % len(colors)]
            
            popup_text = f"{i+1}. {addresses[route_index]}"
            if i == 0:
                popup_text = f"🚩 INICIO: {addresses[route_index]}"
            elif i == len(optimized_route) - 1:
                popup_text = f"🏁 FIN: {addresses[route_index]}"
            
            folium.Marker(
                coord,
                popup=popup_text,
                tooltip=f"Parada {i+1}",
                icon=folium.Icon(color=color, icon='info-sign')
            ).add_to(m)
        
        # Línea de ruta
        route_coords = [coordinates[i] for i in optimized_route]
        folium.PolyLine(
            route_coords,
            weight=3,
            color='red',
            opacity=0.7,
            tooltip="Ruta optimizada"
        ).add_to(m)
        
        return m._repr_html_()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/optimize', methods=['POST'])
def optimize_route():
    try:
        # Verificar que se configuró la API key
        if OPENROUTESERVICE_API_KEY == "TU_API_KEY_AQUI":
            return jsonify({
                'error': 'API key no configurada. Edita app.py y coloca tu API key de OpenRouteService en la línea 18.'
            }), 500
        
        data = request.json
        addresses = data.get('addresses', [])
        start_address = data.get('start_address', '')
        optimization_type = data.get('optimization_type', 'distance')
        
        if len(addresses) < 1:
            return jsonify({'error': 'Se necesita al menos 1 dirección de destino'}), 400
        
        if len(addresses) > 15:
            return jsonify({'error': 'Máximo 15 direcciones para evitar límites de API'}), 400
        
        # Inicializar optimizador
        optimizer = RouteOptimizer(OPENROUTESERVICE_API_KEY)
        all_addresses = [start_address] + addresses
        
        # Geocodificar direcciones
        coordinates = []
        for i, addr in enumerate(all_addresses):
            try:
                coord = optimizer.geocode_address(addr)
                
                if coord is None or len(coord) != 2 or coord[0] is None or coord[1] is None:
                    raise ValueError(f"Coordenada inválida para: {addr}")
                
                coordinates.append(coord)
                
                if i < len(all_addresses) - 1:
                    time.sleep(0.5)  # Evitar rate limiting
                    
            except Exception as e:
                suggestions = []
                if 'caba' in addr.lower():
                    suggestions.append(addr.replace('caba', 'CABA'))
                if 'Bme' in addr:
                    suggestions.append(addr.replace('Bme', 'Bartolomé'))
                
                error_msg = f'Error geocodificando "{addr}": {str(e)}'
                if suggestions:
                    error_msg += f'. Intenta con: {suggestions[0]}'
                
                return jsonify({'error': error_msg}), 400
        
        # Crear matriz de distancias
        distance_matrix = optimizer.create_distance_matrix(coordinates)
        
        # Resolver TSP
        optimized_route = optimizer.solve_tsp(distance_matrix, start_index=0)
        
        # Calcular métricas
        total_distance = 0
        for i in range(len(optimized_route) - 1):
            from_idx = optimized_route[i]
            to_idx = optimized_route[i + 1]
            total_distance += distance_matrix[from_idx][to_idx]
        
        # Estimar tiempo (40 km/h promedio en ciudad)
        estimated_time_hours = total_distance / 40
        
        # Crear visualizaciones
        map_html = optimizer.create_map(coordinates, all_addresses, optimized_route)
        google_maps_link = optimizer.create_google_maps_link(all_addresses, optimized_route)
        
        ordered_addresses = [all_addresses[i] for i in optimized_route]
        
        return jsonify({
            'success': True,
            'optimized_route': ordered_addresses,
            'total_distance_km': round(total_distance, 2),
            'total_time_hours': round(estimated_time_hours, 2),
            'total_time_minutes': round(estimated_time_hours * 60, 1),
            'map_html': map_html,
            'google_maps_link': google_maps_link,
            'optimization_type': optimization_type
        })
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    app.run(debug=False, port=5000)