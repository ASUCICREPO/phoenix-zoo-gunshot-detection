module.exports = (app) => {
  const incident = require("../controllers/incidents.controller.js");

  const router = require("express").Router();

  // get all incidents
  router.get("/", incident.getIncidents);

  app.use("/api/incidents", router);
};
