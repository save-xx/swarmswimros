import requests

def get_items():
    url = "http://localhost:8000/status"
    response = requests.get(url)
    
    if response.status_code == 200:
        status = response.json()
        print(status)
    else:
        print(f"Failed to get items with status code {response.status_code}")

if __name__ == "__main__":
    get_items()