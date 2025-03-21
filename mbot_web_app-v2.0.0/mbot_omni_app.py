from flask import Flask, jsonify, request
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

@app.route('/api/data', methods=['GET'])
def get_data():
    # Temporary, since the backend does not currently do anything.
    data = {'message': 'Hello!'}
    return jsonify(data)

if __name__ == '__main__':
    app.run(debug=True)
