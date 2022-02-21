const express = require("express");
const cors = require('cors');
const app = express();
const path = require('path');

var corsOptions = {
  origin: "http://localhost:3000",
};

app.use(cors(corsOptions));

app.use(express.json());

app.use(express.urlencoded({ extended: true }));

// app.use(express.static(path.join(__dirname, 'client/build')));

// simple route
// app.get("/", (req, res) => {
//   res.json({ message: "Hello from gunshot project!" });
// });

require("./app/routes/incidents.routes")(app);
require("./app/routes/devices.routes")(app);

// app.get('*', (req,res) =>{
//   res.sendFile(path.join(__dirname+'/client/build/index.html'));
// });

const PORT = process.env.PORT || 8080;
app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}.`);
});
